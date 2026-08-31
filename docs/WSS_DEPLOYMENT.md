# DexFull WSS deployment

DexFull's Unity endpoint now uses one-way TLS at
`wss://<robot-ip>:7443/ws`. Existing Unity method names, JSON envelopes,
parameters, responses, heartbeat and stream payloads are unchanged.

## Security boundary

All robots use the same server certificate and private key. Unity stores and
pins that certificate's SHA-256 fingerprint. This prevents passive network
observers from reading requests/responses and makes a certificate substitution
fail closed in a correctly pinned Unity client.

This design does **not** authenticate Unity. A custom client that knows the
protocol can still establish WSS and invoke methods, and compromise of the
shared private key affects every deployed robot. If preventing non-Unity
clients becomes a requirement, add application authentication or mTLS in a
separate versioned change.

## 1. Generate the common certificate once

Run on a trusted release/build machine, not separately on each robot:

```bash
cd ~/ws/dexfull_robot
bash scripts/generate_common_ws_cert.sh ./common-wss-certificate
```

Keep these release artifacts protected:

- `server-cert.pem`: public certificate, also packaged with Unity for pinning.
- `server-key.pem`: fleet-wide secret; never commit it or place it in a public
  download.
- `server-fingerprint.txt`: lower-case SHA-256 pin embedded/imported by Unity.

## 2. Install the same pair on every robot

```bash
install -d -m 700 ~/.config/dexfull/ws-tls
install -m 644 common-wss-certificate/server-cert.pem \
  ~/.config/dexfull/ws-tls/server-cert.pem
install -m 600 common-wss-certificate/server-key.pem \
  ~/.config/dexfull/ws-tls/server-key.pem
```

The default `config/dexfull.yaml` already selects port 7443 and these paths.
Start normally:

```bash
python3 -m dexfull
```

Expected log lines include `WSS enabled` and
`DexFull ... ready wss://0.0.0.0:7443/ws`. Missing or invalid certificate
material stops startup; production does not silently fall back to plaintext.

## 3. Unity connection change

Change only the connection layer:

```csharp
URL = WebSocketTools.URLBuild(socketIP, 7443, "/ws", useTls: true);
```

In Unity's TLS certificate callback, calculate SHA-256 over the peer leaf
certificate DER bytes and compare it, in constant time, with the value shipped
from `server-fingerprint.txt`. Reject missing certificates, mismatches and all
certificate errors that do not end in an exact pin match. Do not implement an
“ignore certificate errors” fallback.

No request method or parameter change is required. Unity sends the same JSON
only after certificate pin validation succeeds.

## 4. Verify before Unity rollout

```bash
python3 scripts/ws_tls_probe.py \
  --url wss://10.2.5.18:7443/ws \
  --server-cert common-wss-certificate/server-cert.pem
```

Also verify that `ws://10.2.5.18:7443/ws` cannot complete a WebSocket
handshake. The old TCP 7000 listener should not exist.

## Development-only plaintext override

For an isolated development machine only, both settings must be explicit:

```yaml
ws:
  allow_insecure_transport: true
  tls:
    enabled: false
```

Never deploy this override to a robot.

## Certificate rotation

Generate into a new empty release directory, update Unity's pin, deploy the new
certificate and key, then restart DexFull. Because the current client holds one
pin, coordinate server and Unity rollout; supporting overlapping old/new pins
in Unity makes staged rotation easier.