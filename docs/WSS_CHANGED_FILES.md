# DexFull WSS change list

Compared with the pre-WSS DexFull source, the Unity JSON/RPC interface is
unchanged. Only the connection layer, its configuration, deployment tools,
tests and documentation changed.

| File | Change |
| --- | --- |
| `dexfull/common/ws_security.py` | New fail-closed TLS policy; loads the common certificate/key, enforces TLS 1.2+, disables TLS compression and logs the server SHA-256 fingerprint. |
| `dexfull/bridge/ws_server.py` | Passes the TLS context into the existing WebSocket listener and reports the actual `wss`/`ws` scheme. |
| `dexfull/app.py` | Builds the TLS policy before listening, changes the fallback port to 7443 and logs the WSS endpoint. |
| `dexfull/config.py` | Changes built-in defaults to WSS/7443 and adds certificate paths plus the fail-closed plaintext switch. |
| `config/dexfull.yaml` | Production WSS configuration using the shared certificate under `~/.config/dexfull/ws-tls/`. |
| `scripts/generate_common_ws_cert.sh` | Generates the one shared certificate/key and Unity SHA-256 pin without overwriting existing key material. |
| `scripts/ws_tls_probe.py` | Connects with certificate pinning and sends an unchanged DexFull RPC request. |
| `scripts/device_recovery_stress.py` | Moves the existing unplug/replug test to WSS and validates the server pin before sending data. |
| `tests/test_ws_security.py` | Covers fail-closed configuration and the explicit development-only plaintext override. |
| `tests/test_ws_tls_integration.py` | Covers real WSS handshake, unchanged heartbeat payloads, no client certificate, and plaintext rejection. |
| `docs/UNITY_API.md` | Changes the documented endpoint to WSS/7443 and adds the pinning requirement. |
| `docs/REAL_ROBOT_1_1.md` | Changes the device recovery command to WSS with its pinned certificate. |
| `docs/WSS_DEPLOYMENT.md` | New certificate generation, robot install, Unity pinning, verification and rotation guide. |
| `pyproject.toml` | Adds `trustme` to test extras for isolated TLS integration certificates. |
| `uv.lock` | Locks the new optional WSS integration-test dependency and current project version. |

Not changed: Unity method names, parameters, response fields, stream payloads,
DDS topics, camera ZMQ transport, XR control IPC, hand-driver protocol and the
existing XR browser certificate helper in `dexfull/common/tls.py`.