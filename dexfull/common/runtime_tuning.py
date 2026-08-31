"""Optional native math-library tuning for each DexFull process."""

from __future__ import annotations

import os


_NATIVE_THREAD_VARIABLES = (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
)


def configure_native_threads(default: int | None = None) -> dict[str, str]:
    """Apply an explicit native-thread override, otherwise keep library defaults.

    CasADi/IPOPT and BLAS may use several CPU cores inside the isolated control
    process.  Forcing every native library to one thread reduced G1 IK
    throughput by roughly half on the robot.  Deployments can still set
    ``DEXFULL_NATIVE_THREADS`` (or the library variables directly), but the
    default is deliberately left untouched.
    """

    requested = os.environ.get("DEXFULL_NATIVE_THREADS")
    if requested is None and default is None:
        return {
            name: os.environ[name]
            for name in _NATIVE_THREAD_VARIABLES
            if name in os.environ
        }
    value = str(max(1, int(requested if requested is not None else default)))
    for name in _NATIVE_THREAD_VARIABLES:
        os.environ.setdefault(name, value)
    os.environ.setdefault("OMP_WAIT_POLICY", "PASSIVE")
    return {name: os.environ[name] for name in _NATIVE_THREAD_VARIABLES}
