"""TLS certificate discovery and first-run setup for the XR web server."""

from __future__ import annotations

import logging
import os
import shutil
import socket
import subprocess
from pathlib import Path
from typing import Optional, Tuple

from .paths import PROJECT_ROOT


logger = logging.getLogger("DexFull.TLS")


def resolve_or_create_tls_pair(
    cert_file: Optional[str] = None,
    key_file: Optional[str] = None,
) -> Tuple[str, str]:
    """Return a usable certificate/key pair, creating a self-signed pair if needed."""
    if cert_file is not None or key_file is not None:
        return _validated_pair(cert_file, key_file, "TeleVuer arguments")

    env_cert = os.environ.get("XR_TELEOP_CERT")
    env_key = os.environ.get("XR_TELEOP_KEY")
    if env_cert is not None or env_key is not None:
        return _validated_pair(env_cert, env_key, "XR_TELEOP_CERT/XR_TELEOP_KEY")

    candidates = (
        Path.home() / ".config" / "dexfull",
        # Preserve the real robot 1.1 certificate location.
        Path.home() / ".config" / "xr_teleoperate",
        PROJECT_ROOT,
    )
    for directory in candidates:
        cert_path = directory / "cert.pem"
        key_path = directory / "key.pem"
        if cert_path.is_file() and key_path.is_file():
            logger.info("Using XR TLS certificate from %s", directory)
            return str(cert_path), str(key_path)

    return _create_self_signed_pair(Path.home() / ".config" / "dexfull")


def _validated_pair(cert_file, key_file, source: str) -> Tuple[str, str]:
    if not cert_file or not key_file:
        raise FileNotFoundError(
            f"{source} must provide both a certificate and a private key"
        )
    cert_path = Path(cert_file).expanduser()
    key_path = Path(key_file).expanduser()
    missing = [str(path) for path in (cert_path, key_path) if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            f"XR TLS file does not exist ({source}): {', '.join(missing)}"
        )
    return str(cert_path), str(key_path)


def _create_self_signed_pair(directory: Path) -> Tuple[str, str]:
    openssl = shutil.which("openssl")
    if openssl is None:
        raise FileNotFoundError(
            "XR TLS certificate is missing and openssl is unavailable. "
            "Install openssl or set XR_TELEOP_CERT and XR_TELEOP_KEY."
        )

    directory.mkdir(parents=True, exist_ok=True)
    cert_path = directory / "cert.pem"
    key_path = directory / "key.pem"
    cert_tmp = directory / "cert.pem.tmp"
    key_tmp = directory / "key.pem.tmp"
    hostname = socket.gethostname().strip() or "dexfull"
    command = [
        openssl,
        "req",
        "-x509",
        "-nodes",
        "-days",
        "3650",
        "-newkey",
        "rsa:2048",
        "-keyout",
        str(key_tmp),
        "-out",
        str(cert_tmp),
        "-subj",
        f"/CN={hostname}",
    ]
    try:
        subprocess.run(
            command,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30.0,
        )
        key_tmp.chmod(0o600)
        cert_tmp.replace(cert_path)
        key_tmp.replace(key_path)
    except Exception as exc:
        for temporary in (cert_tmp, key_tmp):
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
        raise RuntimeError(
            f"Failed to generate XR TLS certificate in {directory}: {exc}"
        ) from exc

    logger.info("Generated XR TLS certificate in %s", directory)
    return str(cert_path), str(key_path)
