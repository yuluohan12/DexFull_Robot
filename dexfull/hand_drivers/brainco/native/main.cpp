#include "stark-sdk.h"
#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> running{true};
std::mutex discovery_mutex;
std::set<std::string> claimed_ports;
constexpr int kFailureWarningThreshold = 25;
constexpr auto kReconnectRampDuration = std::chrono::milliseconds(1000);
constexpr auto kPerformanceLogInterval = std::chrono::seconds(5);
constexpr std::array<int, 5> kReconnectBackoffSeconds{1, 2, 5, 10, 30};

void signal_handler(int) { running = false; }

struct HandConnection {
    DeviceHandler* handle{nullptr};
    CDeviceInfo* info{nullptr};
    uint8_t slave_id{0};
    StarkProtocolType protocol{STARK_PROTOCOL_TYPE_MODBUS};
    std::string port;
};

class HandCommandSubscription final
    : public unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_> {
public:
    using Message = unitree_go::msg::dds_::MotorCmds_;
    explicit HandCommandSubscription(const std::string& topic)
        : unitree::robot::SubscriptionBase<Message>(topic) {}

    Message snapshot() {
        std::lock_guard<std::mutex> lock(this->mutex_);
        return this->msg_;
    }
};

std::vector<std::string> get_available_serial_ports() {
    std::vector<std::string> ports;
    try {
        for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
            const std::string path = entry.path().string();
            if (path.rfind("/dev/ttyUSB", 0) == 0 ||
                path.rfind("/dev/ttyHAND", 0) == 0 ||
                path.rfind("/dev/ttyUN", 0) == 0) {
                ports.push_back(path);
            }
        }
    } catch (const std::exception& exc) {
        spdlog::warn("Failed to enumerate serial ports: {}", exc.what());
    }
    std::sort(ports.begin(), ports.end());
    return ports;
}

void close_connection(HandConnection& connection) {
    if (connection.handle) {
        close_device_handler(connection.handle, connection.protocol);
        connection.handle = nullptr;
    }
    if (connection.info) {
        free_device_info(connection.info);
        connection.info = nullptr;
    }
    if (!connection.port.empty()) {
        std::lock_guard<std::mutex> lock(discovery_mutex);
        claimed_ports.erase(connection.port);
    }
    connection.port.clear();
}

HandConnection find_hand_on_port(const std::string& port,
                                 const std::vector<SkuType>& allowed_skus,
                                 const std::string& hand_name) {
    HandConnection result;
    result.port = port;
    spdlog::info("Scanning for {} hand on {} ...", hand_name, port);
    CDetectedDeviceList* detected =
        stark_auto_detect(true, port.c_str(), STARK_PROTOCOL_TYPE_MODBUS);
    if (!detected || detected->count == 0) {
        free_detected_device_list(detected);
        return result;
    }

    for (uintptr_t index = 0; index < detected->count; ++index) {
        const CDetectedDevice& device = detected->devices[index];
        if (std::find(allowed_skus.begin(), allowed_skus.end(), device.sku_type) ==
            allowed_skus.end()) {
            continue;
        }
        DeviceHandler* handle = init_from_detected(&device);
        if (!handle) continue;
        CDeviceInfo* info = stark_get_device_info(handle, device.slave_id);
        if (!info) {
            close_device_handler(handle, device.protocol);
            continue;
        }
        stark_set_finger_unit_mode(handle, device.slave_id,
                                   FINGER_UNIT_MODE_NORMALIZED);
        result.handle = handle;
        result.info = info;
        result.slave_id = device.slave_id;
        result.protocol = device.protocol;
        spdlog::info(
            "{} hand found on {} (slave=0x{:02x}, sku={}, sn={}, fw={})",
            hand_name, port, device.slave_id, static_cast<int>(info->sku_type),
            info->serial_number ? info->serial_number : "?",
            info->firmware_version ? info->firmware_version : "?");
        break;
    }
    free_detected_device_list(detected);
    return result;
}

HandConnection discover_hand(const std::vector<SkuType>& allowed_skus,
                             const std::string& hand_name) {
    // Vendor discovery opens ports exclusively. Serialize it and reserve the
    // selected port so the two side supervisors cannot claim the same device.
    std::lock_guard<std::mutex> lock(discovery_mutex);
    const auto ports = get_available_serial_ports();
    spdlog::info("Available Serial Ports: {}", fmt::join(ports, ", "));
    for (const auto& port : ports) {
        if (claimed_ports.count(port)) continue;
        HandConnection connection =
            find_hand_on_port(port, allowed_skus, hand_name);
        if (connection.handle) {
            claimed_ports.insert(port);
            return connection;
        }
    }
    return {};
}

bool publish_status(
    const CMotorStatusData* status,
    unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>* state) {
    if (!state->trylock()) return false;
    for (int i = 0; i < 6; ++i) {
        state->msg_.states()[i].q() = status->positions[i] / 1000.f;
        state->msg_.states()[i].dq() = status->speeds[i] / 1000.f;
        state->msg_.states()[i].tau_est() = status->currents[i] / 1000.f;
    }
    state->unlockAndPublish();
    return true;
}

void hand_supervisor(const std::string& side,
                     const std::vector<SkuType>& allowed_skus) {
    auto command = std::make_shared<HandCommandSubscription>(
        "rt/brainco/" + side + "/cmd");
    command->msg_.cmds().resize(6);
    for (auto& finger : command->msg_.cmds()) finger.dq() = 1.f;

    auto state = std::make_unique<
        unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
        "rt/brainco/" + side + "/state");
    state->msg_.states().resize(6);

    std::size_t retry = 0;
    while (running) {
        HandConnection connection = discover_hand(allowed_skus, side);
        if (!connection.handle) {
            const int delay = kReconnectBackoffSeconds[
                std::min(retry, kReconnectBackoffSeconds.size() - 1)];
            ++retry;
            spdlog::warn("{} hand unavailable; retrying in {}s", side, delay);
            for (int tick = 0; running && tick < delay * 10; ++tick)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        retry = 0;
        int failures = 0;
        std::array<uint16_t, 6> reconnect_position{};

        // Measure before the first command. A just-reconnected hand must not
        // jump directly to a stale DDS target.
        CMotorStatusData* initial =
            stark_get_motor_status(connection.handle, connection.slave_id);
        bool command_ready = initial != nullptr;
        if (initial) {
            for (int i = 0; i < 6; ++i)
                reconnect_position[i] = initial->positions[i];
            publish_status(initial, state.get());
            free_motor_status_data(initial);
        } else {
            // Do not close/reopen a valid FTDI handle because the very first
            // status sample was delayed. Wait on this handle and start the
            // safety ramp after the first successful measurement.
            spdlog::warn(
                "{} hand connected but initial status read failed; "
                "keeping the existing serial handle",
                side);
        }
        auto ramp_started = std::chrono::steady_clock::now();
        auto diagnostic_started = ramp_started;
        std::size_t diagnostic_cycles = 0;
        std::size_t diagnostic_status_samples = 0;
        std::size_t diagnostic_published_samples = 0;
        std::size_t diagnostic_publish_drops = 0;
        float diagnostic_max_cycle_ms = 0.f;
        spdlog::info("{} hand ONLINE on {}; command ramp enabled", side,
                     connection.port);

        while (running) {
            const auto started = std::chrono::steady_clock::now();
            // Match the proven source service: deliver the newest command
            // before a synchronous status read can block this serial loop.
            if (command_ready) {
                uint16_t positions[6];
                uint16_t speeds[6];
                const auto desired = command->snapshot();
                const auto ramp_elapsed = started - ramp_started;
                const float alpha = std::clamp(
                    std::chrono::duration<float>(ramp_elapsed).count() /
                        std::chrono::duration<float>(kReconnectRampDuration).count(),
                    0.f, 1.f);
                for (int i = 0; i < 6; ++i) {
                    const float target =
                        std::clamp(desired.cmds()[i].q(), 0.f, 1.f) * 1000.f;
                    positions[i] = static_cast<uint16_t>(
                        reconnect_position[i] +
                        (target - reconnect_position[i]) * alpha);
                    speeds[i] = static_cast<uint16_t>(
                        std::clamp(desired.cmds()[i].dq(), 0.f, 1.f) * 1000.f);
                }
                stark_set_finger_positions_and_speeds(
                    connection.handle, connection.slave_id, positions, speeds, 6);
            }

            CMotorStatusData* status =
                stark_get_motor_status(connection.handle, connection.slave_id);
            if (!status) {
                ++failures;
                // Match the proven source service for transient SDK read
                // failures: keep the live handle and continue delivering
                // commands. Closing a dual-channel FTDI handle merely because
                // 25 reads timed out can reset both interfaces and turn a
                // short scheduling delay into repeated ttyUSB re-enumeration.
                // A physical unplug is different: its device node disappears,
                // at which point the outer supervisor may safely reconnect.
                if (!std::filesystem::exists(connection.port)) {
                    spdlog::warn(
                        "{} hand device node {} disappeared after {} read failures",
                        side, connection.port, failures);
                    break;
                }
                if (failures == kFailureWarningThreshold ||
                    failures % (kFailureWarningThreshold * 10) == 0) {
                    spdlog::warn(
                        "{} hand has {} consecutive status read failures; "
                        "keeping the existing serial handle because {} still exists",
                        side, failures, connection.port);
                }
            } else {
                failures = 0;
                if (!command_ready) {
                    for (int i = 0; i < 6; ++i)
                        reconnect_position[i] = status->positions[i];
                    ramp_started = std::chrono::steady_clock::now();
                    command_ready = true;
                    spdlog::info(
                        "{} hand received its first status sample; "
                        "command ramp enabled",
                        side);
                }
                if (publish_status(status, state.get())) {
                    ++diagnostic_published_samples;
                } else {
                    ++diagnostic_publish_drops;
                }
                free_motor_status_data(status);
                ++diagnostic_status_samples;
            }

            const auto finished = std::chrono::steady_clock::now();
            const auto elapsed = finished - started;
            ++diagnostic_cycles;
            diagnostic_max_cycle_ms = std::max(
                diagnostic_max_cycle_ms,
                std::chrono::duration<float, std::milli>(elapsed).count());
            const auto diagnostic_elapsed = finished - diagnostic_started;
            if (diagnostic_elapsed >= kPerformanceLogInterval) {
                const float seconds = std::max(
                    std::chrono::duration<float>(diagnostic_elapsed).count(),
                    0.001f);
                spdlog::info(
                    "{} hand serial stats: loop_hz={:.1f} read_hz={:.1f} "
                    "publish_hz={:.1f} publish_drops={} max_io_ms={:.1f} "
                    "failures={}",
                    side, diagnostic_cycles / seconds,
                    diagnostic_status_samples / seconds,
                    diagnostic_published_samples / seconds,
                    diagnostic_publish_drops, diagnostic_max_cycle_ms, failures);
                diagnostic_started = finished;
                diagnostic_cycles = 0;
                diagnostic_status_samples = 0;
                diagnostic_published_samples = 0;
                diagnostic_publish_drops = 0;
                diagnostic_max_cycle_ms = 0.f;
            }
            const auto period = std::chrono::milliseconds(20);
            if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
        }

        if (running && !connection.port.empty()) {
            spdlog::warn("{} hand DISCONNECTED because device node {} disappeared",
                         side, connection.port);
        }
        close_connection(connection);
    }
    spdlog::info("{} hand supervisor exiting", side);
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto vm = param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(
        0, vm["network_interface"].as<std::string>());
    init_logging(LogLevel::LOG_LEVEL_ERROR);

    // Start both supervisors even when no serial device exists at boot.
    std::thread left(hand_supervisor, "left",
                     std::vector<SkuType>{SKU_TYPE_SMALL_LEFT,
                                          SKU_TYPE_MEDIUM_LEFT});
    std::thread right(hand_supervisor, "right",
                      std::vector<SkuType>{SKU_TYPE_SMALL_RIGHT,
                                           SKU_TYPE_MEDIUM_RIGHT});
    left.join();
    right.join();
    spdlog::info("BrainCo hand service exited.");
    return 0;
}
