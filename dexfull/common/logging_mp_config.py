"""Process-wide, idempotent configuration for :mod:`logging_mp`.

``logging_mp`` starts its dispatcher as soon as either ``basicConfig`` or
``getLogger`` is called.  DexFull loads control and hand-driver modules lazily,
so their import order must not decide whether a later ``basicConfig`` raises.
"""

from __future__ import annotations

import logging
import threading
from types import ModuleType


_lock = threading.Lock()
_configured = False


def configure_logging_mp(logging_module: ModuleType | None = None) -> bool:
    """Configure ``logging_mp`` once.

    Returns ``True`` when this call performed ``basicConfig``.  A logger may
    already have started the logging dispatcher when DexFull is embedded or a
    plugin is imported directly; that state is valid and is treated as already
    configured instead of aborting XR startup.
    """

    global _configured

    with _lock:
        if _configured:
            return False

        if logging_module is None:
            import logging_mp as logging_module

        configured_now = True
        try:
            logging_module.basicConfig(level=logging_module.INFO)
        except RuntimeError as exc:
            if "already been started" not in str(exc).lower():
                raise
            configured_now = False

        _configured = True
        return configured_now


def configure_control_process_logging(
    logging_module: ModuleType | None = None,
) -> None:
    """Use synchronous process-local logging in the isolated control tree.

    The control process intentionally forks vendor/retargeting workers after it
    has started. ``logging_mp`` owns a background dispatcher and a
    multiprocessing queue; inheriting those thread-owned locks across ``fork``
    can permanently block a worker (and eventually the main control loop) in
    ``Queue.put``. Mapping its small compatibility surface to stdlib logging
    before importing the control stack keeps existing call sites unchanged and
    removes the inherited queue entirely.
    """

    global _configured

    if logging_module is None:
        import logging_mp as logging_module

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        force=True,
    )
    logging_module.getLogger = logging.getLogger
    with _lock:
        # session.py calls configure_logging_mp again at import time. Marking
        # this process configured makes that call a no-op instead of starting
        # the unsafe dispatcher after the compatibility mapping above.
        _configured = True
