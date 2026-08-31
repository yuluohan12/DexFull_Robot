#!/usr/bin/env python3
"""Probe DexFull over WSS while pinning the common server certificate."""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import hmac
import json
import ssl
import time
import uuid

import websockets


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DexFull WSS probe")
    parser.add_argument("--url", required=True, help="wss://<robot-ip>:7443/ws")
    parser.add_argument("--server-cert", required=True, help="Pinned common server-cert.pem")
    parser.add_argument("--method", default="get_status", help="Existing RPC method to probe")
    return parser.parse_args()


def pinned_client_context() -> ssl.SSLContext:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    return context


def expected_fingerprint(cert_file: str) -> bytes:
    with open(cert_file, "r", encoding="ascii") as handle:
        der = ssl.PEM_cert_to_DER_cert(handle.read())
    return hashlib.sha256(der).digest()


def verify_peer_pin(websocket, expected: bytes) -> None:
    ssl_object = websocket.transport.get_extra_info("ssl_object")
    actual_der = ssl_object.getpeercert(binary_form=True) if ssl_object else None
    actual = hashlib.sha256(actual_der or b"").digest()
    if not actual_der or not hmac.compare_digest(actual, expected):
        raise ssl.SSLError("server certificate SHA-256 pin mismatch")


async def run(args: argparse.Namespace) -> None:
    request_id = str(uuid.uuid4())
    payload = {
        "type": "request",
        "id": request_id,
        "method": args.method,
        "timestamp": int(time.time() * 1000),
        "data": {},
    }
    async with websockets.connect(
        args.url,
        ssl=pinned_client_context(),
        max_size=2**20,
    ) as websocket:
        # Never send application data until the common certificate pin matches.
        verify_peer_pin(websocket, expected_fingerprint(args.server_cert))
        await websocket.send(json.dumps(payload, ensure_ascii=False))
        while True:
            raw = await asyncio.wait_for(websocket.recv(), timeout=10.0)
            message = json.loads(raw)
            print(json.dumps(message, ensure_ascii=False, indent=2))
            if message.get("id") == request_id:
                return


def main() -> None:
    asyncio.run(run(parse_args()))


if __name__ == "__main__":
    main()
