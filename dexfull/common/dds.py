"""One DDS domain initialization point per DexFull process."""

from __future__ import annotations

import logging
import os
import threading


_lock = threading.RLock()
_initialized = False
_settings = None
logger = logging.getLogger("DexFull.DDS")


def initialize_dds(domain_id: int = 0, network_interface=None) -> None:
    global _initialized, _settings
    network_interface = network_interface or os.environ.get("DEXFULL_DDS_INTERFACE")
    requested = (int(domain_id), network_interface or None)
    with _lock:
        if _initialized:
            if requested != _settings:
                raise RuntimeError(
                    "DDS is already initialized with different settings: "
                    f"active={_settings}, requested={requested}"
                )
            return

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(
            requested[0],
            networkInterface=requested[1],
        )
        _settings = requested
        _initialized = True
        logger.info(
            "DDS initialized: domain=%s, interface=%s",
            requested[0],
            requested[1] or "auto",
        )


def dds_status() -> dict:
    with _lock:
        return {
            "initialized": _initialized,
            "domain_id": None if _settings is None else _settings[0],
            "network_interface": None if _settings is None else _settings[1],
        }
