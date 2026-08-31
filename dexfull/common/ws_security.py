"""TLS policy for DexFull's Unity-facing WebSocket server.

TLS is intentionally kept below the existing JSON/RPC protocol, so enabling
WSS doesn't change Unity method names, envelopes, or payload fields.
"""

from __future__ import annotations

import hashlib
import logging
import os
import ssl
from dataclasses import dataclass
from pathlib import Path
from typing import Any

logger = logging.getLogger("DexFull.WSSecurity")


def _resolve_path(value: Any, base_dir: Path | None) -> Path | None:
    text = str(value or "").strip()
    if not text:
        return None
    path = Path(os.path.expandvars(os.path.expanduser(text)))
    if not path.is_absolute() and base_dir is not None:
        path = base_dir / path
    return path.resolve()


def _certificate_fingerprint_from_pem(path: Path) -> str:
    der = ssl.PEM_cert_to_DER_cert(path.read_text(encoding="ascii"))
    return hashlib.sha256(der).hexdigest()


@dataclass(frozen=True)
class WebSocketTlsPolicy:
    """Server TLS context and observable connection scheme."""

    context: ssl.SSLContext | None
    server_certificate_sha256: str | None = None

    @property
    def enabled(self) -> bool:
        return self.context is not None

    @property
    def scheme(self) -> str:
        return "wss" if self.enabled else "ws"

    @classmethod
    def from_ws_config(
        cls,
        ws_config: dict[str, Any] | None,
        *,
        config_path: str | Path | None = None,
    ) -> "WebSocketTlsPolicy":
        ws_config = ws_config or {}
        tls = ws_config.get("tls") or {}
        if not isinstance(tls, dict):
            raise ValueError("ws.tls must be a mapping")

        if not bool(tls.get("enabled", False)):
            if not bool(ws_config.get("allow_insecure_transport", False)):
                raise ValueError(
                    "insecure ws:// transport is disabled; configure ws.tls or "
                    "explicitly set ws.allow_insecure_transport=true for isolated "
                    "development only"
                )
            logger.warning("INSECURE DEVELOPMENT MODE: Unity endpoint uses plaintext ws://")
            return cls(None)

        base_dir = None
        if config_path:
            base_dir = Path(config_path).expanduser().resolve().parent

        cert_file = _resolve_path(tls.get("cert_file"), base_dir)
        key_file = _resolve_path(tls.get("key_file"), base_dir)
        if cert_file is None or not cert_file.is_file():
            raise FileNotFoundError(f"WSS server certificate not found: {cert_file}")
        if key_file is None or not key_file.is_file():
            raise FileNotFoundError(f"WSS server private key not found: {key_file}")

        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        if hasattr(ssl, "OP_NO_COMPRESSION"):
            context.options |= ssl.OP_NO_COMPRESSION
        context.verify_mode = ssl.CERT_NONE
        context.load_cert_chain(str(cert_file), str(key_file))

        fingerprint = _certificate_fingerprint_from_pem(cert_file)
        logger.info(
            "WSS enabled: server certificate SHA-256=%s; client certificate not required",
            fingerprint,
        )
        return cls(context, fingerprint)