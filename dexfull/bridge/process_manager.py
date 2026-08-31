"""
进程管理器模块。

管理 DexFull 中需要进程隔离的硬件服务完整生命周期：
- 启动 (subprocess.Popen)
- 优雅停止 (SIGTERM → timeout → SIGKILL)
- 健康检查 (定期 poll)
- 可选自动重启
- 状态变化通知回调
"""

import os
import signal
import subprocess
import time
import logging
import threading
from collections import deque
from enum import Enum, auto
from typing import Optional, Callable, Dict, List

logger = logging.getLogger("TeleopBridge.ProcessManager")


class ProcessState(Enum):
    """子进程状态枚举"""
    STOPPED = auto()    # 未启动
    STARTING = auto()   # 正在启动
    RUNNING = auto()    # 运行中
    STOPPING = auto()   # 正在停止
    CRASHED = auto()    # 意外崩溃
    FATAL = auto()      # 启动配置等不可恢复错误，等待显式修复/启动
    DEGRADED = auto()   # 崩溃过密，冷却后继续尝试恢复

    def __str__(self):
        return self.name


class ManagedProcess:
    """
    被管理的单个子进程。

    封装 subprocess.Popen，提供启动、停止、健康检查功能。
    支持 Unix 进程组管理（os.setsid），确保 kill 时子进程也被清理。
    """

    def __init__(self, name: str, cmd: List[str], cwd: str = None,
                 env: dict = None, auto_restart: bool = False,
                 max_restarts: int = 3, start_timeout: float = 15.0,
                 log_path: str = None, cpu_affinity=None,
                 restart_window_seconds: float = 60.0,
                 stable_reset_seconds: float = 60.0,
                 degraded_retry_seconds: float = 30.0):
        """
        Args:
            name: 进程名称 (用于日志)
            cmd: 启动命令列表 (e.g. ["python", "script.py", "--flag"])
            cwd: 工作目录
            env: 额外环境变量
            auto_restart: 崩溃时是否自动重启
            max_restarts: 最大自动重启次数
            start_timeout: 启动超时秒数
        """
        self.name = name
        self.cmd = cmd
        self.cwd = cwd or os.getcwd()
        self.env = env or {}
        self.log_path = log_path
        self._log_file = None
        self.auto_restart = auto_restart
        self.max_restarts = max_restarts
        self.start_timeout = start_timeout
        self.cpu_affinity = self._normalize_cpu_affinity(cpu_affinity)
        self.restart_window_seconds = max(1.0, float(restart_window_seconds))
        self.stable_reset_seconds = max(1.0, float(stable_reset_seconds))
        self.degraded_retry_seconds = max(1.0, float(degraded_retry_seconds))

        self.state = ProcessState.STOPPED
        self.process: Optional[subprocess.Popen] = None
        self.pid: Optional[int] = None
        self.restart_count = 0
        self.last_start_time = 0.0
        self.stop_requested = False
        self.last_crash_time = 0.0
        self._crash_times = deque()
        self.next_retry_at = 0.0
        self._stable_reset_done = False
        self.last_error = None

    @staticmethod
    def _normalize_cpu_affinity(value):
        if value is None or value == "":
            return None
        if isinstance(value, int):
            return [value]
        if isinstance(value, str):
            value = value.strip()
            if not value:
                return None
            return [int(part.strip()) for part in value.split(",") if part.strip()]
        return [int(v) for v in value]

    def _apply_cpu_affinity(self):
        if not self.cpu_affinity or not self.pid:
            return
        try:
            import psutil
            psutil.Process(self.pid).cpu_affinity(self.cpu_affinity)
            logger.info("%s CPU affinity set to %s", self.name, self.cpu_affinity)
        except Exception as e:
            logger.warning("%s failed to set CPU affinity %s: %s", self.name, self.cpu_affinity, e)

    # ----------------------------------------------------------
    # 启动/停止
    # ----------------------------------------------------------

    def start(self) -> bool:
        """启动子进程

        Returns:
            bool: 是否成功启动
        """
        if self.process and self.process.poll() is None:
            logger.warning(f"{self.name} 已在运行, PID={self.pid}")
            self.state = ProcessState.RUNNING
            return True

        self.state = ProcessState.STARTING
        self.stop_requested = False
        self.last_error = None

        try:
            if self.cwd and not os.path.isdir(self.cwd):
                self.state = ProcessState.FATAL
                self.last_error = f"cwd does not exist: {self.cwd}"
                logger.error("%s start failed: %s", self.name, self.last_error)
                return False
            # 合并环境变量（父进程环境 + 自定义环境）
            merged_env = os.environ.copy()
            merged_env.update(self.env)
            stdout_target = subprocess.DEVNULL
            if self.log_path:
                os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
                self._log_file = open(self.log_path, "ab", buffering=0)
                stdout_target = self._log_file

            self.process = subprocess.Popen(
                self.cmd,
                cwd=self.cwd,
                env=merged_env,
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE,
                stdout=stdout_target,
                stderr=subprocess.STDOUT,
                # Unix: 创建新进程组，方便 kill 整个组
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None,
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
            )
            self.pid = self.process.pid
            self._apply_cpu_affinity()
            self.last_start_time = time.time()
            self._stable_reset_done = False
            self.state = ProcessState.RUNNING
            logger.info(f"{self.name} 已启动, PID={self.pid}, "
                        f"CMD={' '.join(self.cmd)}")
            return True
        except FileNotFoundError as e:
            self.state = ProcessState.FATAL
            self.last_error = str(e)
            logger.error(f"{self.name} 启动失败 (文件不存在): {e}")
            return False
        except Exception as e:
            self.state = ProcessState.FATAL
            self.last_error = str(e)
            logger.error(f"{self.name} 启动失败: {e}")
            return False

    def _close_log_file(self):
        if self._log_file:
            try:
                self._log_file.close()
            except Exception:
                pass
            self._log_file = None

    def _read_log_tail(self, limit=4096) -> str:
        if not self.log_path or not os.path.exists(self.log_path):
            return ""
        try:
            with open(self.log_path, "rb") as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - limit), os.SEEK_SET)
                return f.read().decode("utf-8", errors="replace")
        except Exception as e:
            return f"<failed to read log tail: {e}>"

    def stop(self, timeout: float = 10.0) -> bool:
        """停止子进程

        策略: 先 SIGTERM 优雅停止，超时后 SIGKILL 强制杀死。

        Args:
            timeout: 等待优雅停止的超时秒数

        Returns:
            bool: 是否成功停止
        """
        if not self.process or self.process.poll() is not None:
            self.state = ProcessState.STOPPED
            self.pid = None
            self._close_log_file()
            return True

        self.state = ProcessState.STOPPING
        self.stop_requested = True

        try:
            pgid = None
            if os.name == "nt":
                logger.info(f"Terminate {self.name} (PID={self.pid})")
                self.process.terminate()
            else:
                pgid = os.getpgid(self.process.pid)

            # 先尝试 SIGTERM
            logger.info(f"发送 SIGTERM 到 {self.name} (PID={self.pid})")
            if pgid is not None:
                os.killpg(pgid, signal.SIGTERM)

            # 等待进程退出
            deadline = time.time() + timeout
            while time.time() < deadline:
                if self.process.poll() is not None:
                    self.state = ProcessState.STOPPED
                    self.pid = None
                    logger.info(f"{self.name} 已优雅停止")
                    return True
                time.sleep(0.1)

            # 超时，SIGKILL
            logger.warning(f"{self.name} 超时未响应 SIGTERM "
                           f"({timeout}s)，发送 SIGKILL")
            if os.name == "nt":
                self.process.kill()
            else:
                os.killpg(pgid, signal.SIGKILL)
            self.process.wait()
            self.state = ProcessState.STOPPED
            self.pid = None
            logger.info(f"{self.name} 已强制停止")
            return True

        except ProcessLookupError:
            # 进程已结束
            self.state = ProcessState.STOPPED
            self.pid = None
            return True
        except Exception as e:
            logger.error(f"停止 {self.name} 失败: {e}")
            self.state = ProcessState.FATAL
            return False

    # ----------------------------------------------------------
    # 健康检查
    # ----------------------------------------------------------

    def check_health(self) -> ProcessState:
        """
        检查进程健康状况。

        Returns:
            ProcessState: 当前进程状态
                - RUNNING: 正常运行
                - STOPPED: 已按计划停止
                - CRASHED: 意外崩溃
        """
        # FATAL is reserved for non-recoverable launch/configuration errors.
        if self.state == ProcessState.FATAL:
            return self.state

        if self.state == ProcessState.DEGRADED:
            if self.stop_requested:
                self.state = ProcessState.STOPPED
            elif self.auto_restart and time.time() >= self.next_retry_at:
                self.state = ProcessState.CRASHED
            return self.state

        if self.process is None:
            self.state = ProcessState.STOPPED
            return self.state

        retcode = self.process.poll()
        if retcode is None:
            # 进程仍在运行
            self.state = ProcessState.RUNNING
            now = time.time()
            if (
                not self._stable_reset_done
                and now - self.last_start_time >= self.stable_reset_seconds
            ):
                self._crash_times.clear()
                self.restart_count = 0
                self._stable_reset_done = True
        elif self.stop_requested:
            # 按计划退出
            self.state = ProcessState.STOPPED
            self.pid = None
        
        elif not self.stop_requested:
            if self.state == ProcessState.CRASHED:
                return self.state
            self._close_log_file()
            log_tail = self._read_log_tail()
            self.last_error = f"process exited with code {retcode}"
            if log_tail:
                self.last_error += f"\n--- log tail ---\n{log_tail}"
            if not self.auto_restart:
                self.state = ProcessState.FATAL
                return self.state
            now = time.time()
            self.last_crash_time = now
            while self._crash_times and now - self._crash_times[0] > self.restart_window_seconds:
                self._crash_times.popleft()
            self._crash_times.append(now)
            self.restart_count = len(self._crash_times)
            self.pid = None
            if self.restart_count > self.max_restarts:
                self.state = ProcessState.DEGRADED
                self.next_retry_at = now + self.degraded_retry_seconds
                logger.error(
                    "%s crashed %d times in %.0fs; degraded for %.0fs before retry",
                    self.name,
                    self.restart_count,
                    self.restart_window_seconds,
                    self.degraded_retry_seconds,
                )
            else:
                self.state = ProcessState.CRASHED
        return self.state

    def should_auto_restart(self) -> bool:
        """判断是否应该自动重启"""
        if self.state == ProcessState.FATAL:
            return False

        if not self.auto_restart:
            return False

        return self.state == ProcessState.CRASHED

    # ----------------------------------------------------------
    # 输出读取
    # ----------------------------------------------------------

    def read_stdout(self) -> str:
        """读取进程 stdout（非阻塞）"""
        if self.process and self.process.stdout:
            try:
                return self.process.stdout.read1(4096).decode("utf-8", errors="replace")
            except Exception:
                return ""

    def read_stderr(self) -> str:
        """读取进程 stderr（非阻塞）"""
        if self.process and self.process.stderr:
            try:
                return self.process.stderr.read1(4096).decode("utf-8", errors="replace")
            except Exception:
                return ""

    # ----------------------------------------------------------
    # 信息
    # ----------------------------------------------------------

    def get_info(self) -> dict:
        """获取进程信息字典"""
        return {
            "service": self.name,
            "state": self.state.name,
            "pid": self.pid,
            "restart_count": self.restart_count,
            "uptime": (time.time() - self.last_start_time)
                      if self.last_start_time > 0 else 0,
            "auto_restart": self.auto_restart,
            "cmd": self.cmd,
            "cwd": self.cwd,
            "log_path": self.log_path,
            "log_tail": self._read_log_tail(),
            "last_error": self.last_error,
            "next_retry_at": self.next_retry_at,
        }


class ProcessManager:
    """
    进程管理器，统一管理多个子进程的生命周期。

    管理对象 (在 config.py 中配置):
    - teleimager: 图像服务
    - brainco: BrainCo 厂商 SDK 隔离进程
    """

    def __init__(self, process_configs: dict):
        """
        Args:
            process_configs: 进程配置字典，格式见 config.py CONFIG["processes"]
        """
        self._services: Dict[str, ManagedProcess] = {}
        self._health_thread: Optional[threading.Thread] = None
        self._running = False
        self._on_state_change: Optional[Callable] = None

        for name, cfg in process_configs.items():
            self._services[name] = ManagedProcess(
                name=name,
                cmd=cfg.get("cmd", []),
                cwd=cfg.get("cwd"),
                env=cfg.get("env"),
                auto_restart=cfg.get("auto_restart", False),
                max_restarts=cfg.get("max_restarts", 3),
                start_timeout=cfg.get("start_timeout", 15.0),
                log_path=cfg.get("log_path"),
                cpu_affinity=cfg.get("cpu_affinity"),
                restart_window_seconds=cfg.get("restart_window_seconds", 60.0),
                stable_reset_seconds=cfg.get("stable_reset_seconds", 60.0),
                degraded_retry_seconds=cfg.get("degraded_retry_seconds", 30.0),
            )

    @property
    def services(self) -> Dict[str, ManagedProcess]:
        return self._services

    def set_state_callback(self, callback: Callable):
        """设置状态变化回调（由 Bridge 主类传入，推送到 WebSocket）"""
        self._on_state_change = callback

    # ----------------------------------------------------------
    # 对外命令接口
    # ----------------------------------------------------------

    def start_service(self, name: str) -> dict:
        """启动指定服务

        Args:
            name: 服务名称（例如 "teleimager" / "brainco"）

        Returns:
            dict: {"status": "ok"/"error", "service": name,
                   "pid": int/None, "state": str}
        """
        proc = self._services.get(name)
        if not proc:
            return {"status": "error", "msg": f"未知服务: {name}"}

        # 外部显式启动属于一次新的恢复周期。
        # 自动重启由健康检查线程直接调用proc.start()，不会走这里。
        if proc.state in (ProcessState.STOPPED, ProcessState.FATAL, ProcessState.DEGRADED):
            proc.restart_count = 0
            proc.last_crash_time = 0.0
            proc._crash_times.clear()
            proc.next_retry_at = 0.0

        ok = proc.start()
        result = {
            "status": "ok" if ok else "error",
            "service": name,
            "pid": proc.pid,
            "state": proc.state.name,
        }
        if not ok:
            result["msg"] = f"{name} 启动失败"
        if not ok:
            result["error"] = proc.last_error
            result["cwd"] = proc.cwd
            result["cmd"] = proc.cmd
            result["log_path"] = proc.log_path
            result["log_tail"] = proc._read_log_tail()
        self._notify_state(name, proc.state)
        return result

    def stop_service(self, name: str, timeout: float = 10.0) -> dict:
        """停止指定服务

        Args:
            name: 服务名称
            timeout: SIGTERM 等待超时秒数

        Returns:
            dict: {"status": "ok"/"error", ...}
        """
        proc = self._services.get(name)
        if not proc:
            return {"status": "error", "msg": f"未知服务: {name}"}

        ok = proc.stop(timeout=timeout)
        result = {
            "status": "ok" if ok else "error",
            "service": name,
            "state": proc.state.name,
        }
        self._notify_state(name, proc.state)
        return result

    def get_status(self, name: str = None) -> dict:
        """获取服务状态

        Args:
            name: 服务名称，为 None 时返回所有服务状态

        Returns:
            dict: 服务状态信息
        """
        if name:
            proc = self._services.get(name)
            if not proc:
                return {"status": "error", "msg": f"未知服务: {name}"}
            return proc.get_info()
        else:
            return {
                svc_name: proc.get_info()
                for svc_name, proc in self._services.items()
            }

    def restart_service(self, name: str, timeout: float = 10.0) -> dict:
        """重启指定服务"""
        self.stop_service(name, timeout=timeout)
        time.sleep(0.5)
        return self.start_service(name)

    def is_running(self, name: str) -> bool:
        """判断服务是否在运行"""
        proc = self._services.get(name)
        if not proc:
            return False
        proc.check_health()
        return proc.state == ProcessState.RUNNING

    # ----------------------------------------------------------
    # 健康检查循环
    # ----------------------------------------------------------

    def _health_loop(self):
        """定期检查所有子进程的健康状况 (1Hz)"""
        while self._running:
            for name, proc in self._services.items():
                old_state = proc.state
                new_state = proc.check_health()

                # 自动重启
                if new_state == ProcessState.CRASHED:
                    if proc.should_auto_restart():
                        logger.info(
                            f"{name} 自动重启中 "
                            f"(第 {proc.restart_count} 次)..."
                        )
                        proc.start()
                        new_state = proc.state

                if old_state != new_state:
                    self._notify_state(name, new_state)

            time.sleep(1.0)

    def start_monitoring(self):
        """启动健康检查线程"""
        self._running = True
        self._health_thread = threading.Thread(
            target=self._health_loop, daemon=True
        )
        self._health_thread.start()
        logger.info("进程健康检查已启动 (1Hz)")

    def stop_monitoring(self):
        """停止健康检查"""
        self._running = False
        logger.info("进程健康检查已停止")

    def stop_all(self, timeout: float = 10.0):
        """停止所有子进程"""
        logger.info("正在停止所有子进程...")
        for name, proc in self._services.items():
            if proc.state in (ProcessState.RUNNING, ProcessState.STARTING,
                              ProcessState.CRASHED):
                proc.stop(timeout=timeout)
        logger.info("所有子进程已停止")

    # ----------------------------------------------------------
    # 回调通知
    # ----------------------------------------------------------

    def _notify_state(self, name: str, state: ProcessState):
        """通知外部状态变化"""
        if self._on_state_change:
            try:
                self._on_state_change(name, state)
            except Exception as e:
                logger.error(f"状态回调异常: {e}")
