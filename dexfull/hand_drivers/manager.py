from __future__ import annotations

import logging
import time

from .base import ExternalHandDriver
from .registry import get_hand_plugin


logger = logging.getLogger("DexFull.HandDriver")


class HandDriverManager:
    """Owns optional isolated hardware services for the selected hand plugin."""

    def __init__(self, process_manager, config: dict, selected_type=None):
        self.process_manager = process_manager
        self.config = config or {}
        self.selected_type = None if selected_type is None else str(selected_type).lower()
        self.plugin = get_hand_plugin(self.selected_type)
        self._driver = None
        if self.plugin is not None:
            cfg = self.config.get(self.plugin.name, {})
            if not cfg.get("enabled", True):
                raise RuntimeError(f"selected hand driver is disabled: {self.plugin.name}")
            isolation = cfg.get("isolation", self.plugin.isolation)
            if isolation == "external_process":
                self._driver = ExternalHandDriver(process_manager, self.plugin)

    def start(self) -> dict:
        if self._driver is None:
            return {"status": "ok", "driver": self.selected_type, "isolation": "in_process"}
        return self._driver.start()

    def prepare(self) -> dict:
        """Load the selected control adapter before hardware startup timing begins."""
        if self.plugin is None or self.plugin.prepare_factory is None:
            return {"status": "ok", "driver": self.selected_type, "prepared": True}
        started = time.monotonic()
        logger.info("Preloading %s control module...", self.selected_type)
        try:
            self.plugin.prepare_factory()
        except Exception as exc:
            logger.exception("Failed to preload %s control module", self.selected_type)
            return {
                "status": "error",
                "driver": self.selected_type,
                "prepared": False,
                "msg": f"hand driver preload failed: {exc}",
            }
        elapsed_ms = round((time.monotonic() - started) * 1000.0, 1)
        logger.info(
            "%s control module preloaded in %.1f ms",
            self.selected_type,
            elapsed_ms,
        )
        return {
            "status": "ok",
            "driver": self.selected_type,
            "prepared": True,
            "elapsed_ms": elapsed_ms,
        }

    def stop(self) -> dict:
        if self._driver is None:
            return {"status": "ok", "driver": self.selected_type}
        return self._driver.stop()

    def restart(self) -> dict:
        if self._driver is None:
            return {"status": "ok", "driver": self.selected_type}
        return self._driver.restart()

    def status(self) -> dict:
        if self.plugin is None:
            return {"driver": None, "isolation": "none"}
        if self._driver is None:
            return {"driver": self.plugin.name, "isolation": "in_process"}
        return self._driver.status()
