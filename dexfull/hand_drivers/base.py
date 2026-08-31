from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional


@dataclass
class HandControlContext:
    values: Dict[str, Any] = field(default_factory=dict)

    def get(self, name: str, default=None):
        return self.values.get(name, default)


@dataclass(frozen=True)
class HandPlugin:
    name: str
    joint_names_left: tuple
    joint_names_right: tuple
    isolation: str
    control_factory: Callable[[str, bool], HandControlContext]
    service_name: Optional[str] = None
    collector_factory: Optional[Callable[[], Any]] = None
    prepare_factory: Optional[Callable[[], Any]] = None


class ExternalHandDriver:
    def __init__(self, process_manager, plugin: HandPlugin):
        if not plugin.service_name:
            raise ValueError(f"external hand plugin has no service: {plugin.name}")
        self.process_manager = process_manager
        self.plugin = plugin

    def start(self) -> dict:
        return self.process_manager.start_service(self.plugin.service_name)

    def stop(self) -> dict:
        return self.process_manager.stop_service(self.plugin.service_name)

    def restart(self) -> dict:
        return self.process_manager.restart_service(self.plugin.service_name)

    def status(self) -> dict:
        return {
            "driver": self.plugin.name,
            "isolation": self.plugin.isolation,
            "service": self.plugin.service_name,
            "process": self.process_manager.get_status(self.plugin.service_name),
        }
