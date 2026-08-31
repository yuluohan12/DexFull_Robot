"""Observe manual unplug/replug recovery through the unchanged Unity WS API."""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import hmac
import json
import ssl
import time
import uuid
from pathlib import Path

import websockets


def _expected_certificate_fingerprint(cert_file: str) -> bytes:
    with open(cert_file, "r", encoding="ascii") as handle:
        der = ssl.PEM_cert_to_DER_cert(handle.read())
    return hashlib.sha256(der).digest()


def _tls_context() -> ssl.SSLContext:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    return context


async def run(url: str, server_cert: str, duration: float, expected: set[str]) -> int:
    disconnected = set()
    recovered = set()
    deadline = time.monotonic() + duration
    expected_fingerprint = _expected_certificate_fingerprint(server_cert)
    async with websockets.connect(
        url,
        ssl=_tls_context(),
        ping_interval=5,
        ping_timeout=5,
    ) as socket:
        ssl_object = socket.transport.get_extra_info("ssl_object")
        actual_der = ssl_object.getpeercert(binary_form=True) if ssl_object else None
        actual_fingerprint = hashlib.sha256(actual_der or b"").digest()
        if not actual_der or not hmac.compare_digest(
            actual_fingerprint, expected_fingerprint
        ):
            raise ssl.SSLError("server certificate SHA-256 pin mismatch")
        print(f"connected: {url}; unplug and reconnect the target devices now")
        while time.monotonic() < deadline:
            timeout = min(2.0, deadline - time.monotonic())
            try:
                raw = await asyncio.wait_for(socket.recv(), timeout=timeout)
            except asyncio.TimeoutError:
                await socket.send(json.dumps({
                    "type": "ping",
                    "id": str(uuid.uuid4()),
                    "timestamp": int(time.time() * 1000),
                }))
                continue
            message = json.loads(raw)
            data = message.get("data") or {}
            device = data.get("device")
            if message.get("type") == "error" and data.get("code") == "DEVICE_DISCONNECTED":
                disconnected.add(device)
                print(f"DISCONNECTED {device}: {message.get('error_tip')}")
            elif message.get("type") == "event" and data.get("code") == "DEVICE_RECOVERED":
                recovered.add(device)
                print(f"RECOVERED    {device}")

    targets = expected or disconnected
    missing_disconnect = targets - disconnected
    missing_recovery = targets - recovered
    print(f"disconnect events={sorted(disconnected)}")
    print(f"recovery events={sorted(recovered)}")
    if missing_disconnect or missing_recovery:
        print(
            f"FAILED missing_disconnect={sorted(missing_disconnect)} "
            f"missing_recovery={sorted(missing_recovery)}"
        )
        return 1
    print("PASSED: every observed/expected device disconnected and recovered")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="wss://10.2.5.18:7443/ws")
    parser.add_argument(
        "--server-cert",
        default="~/.config/dexfull/ws-tls/server-cert.pem",
        help="common server certificate pinned by SHA-256",
    )
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument(
        "--expect",
        action="append",
        default=[],
        help="device name; repeat for head_camera, left_hand, etc.",
    )
    args = parser.parse_args()
    server_cert = str(Path(args.server_cert).expanduser())
    raise SystemExit(
        asyncio.run(run(args.url, server_cert, args.duration, set(args.expect)))
    )


if __name__ == "__main__":
    main()
