#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [output-dir]" >&2
  exit 2
fi

command -v openssl >/dev/null 2>&1 || {
  echo "openssl is required" >&2
  exit 1
}

output_dir="${1:-./common-wss-certificate}"
mkdir -p -- "$output_dir"
output_dir="$(cd -- "$output_dir" && pwd -P)"
umask 077

for name in server-cert.pem server-key.pem server-fingerprint.txt; do
  if [[ -e "$output_dir/$name" ]]; then
    echo "Refusing to overwrite existing certificate material: $output_dir/$name" >&2
    echo "Use a new empty directory when rotating the common certificate." >&2
    exit 1
  fi
done

temp_dir="$(mktemp -d)"
cleanup() {
  rm -rf -- "$temp_dir"
}
trap cleanup EXIT

cat >"$temp_dir/server-ext.cnf" <<'EOF'
basicConstraints=critical,CA:FALSE
keyUsage=critical,digitalSignature,keyEncipherment
extendedKeyUsage=serverAuth
subjectAltName=DNS:dexfull.local
EOF

openssl genpkey -algorithm RSA -pkeyopt rsa_keygen_bits:3072 \
  -out "$output_dir/server-key.pem"
openssl req -new -key "$output_dir/server-key.pem" \
  -subj "/CN=DexFull Common WSS Server" \
  -out "$temp_dir/server.csr"
openssl x509 -req -sha256 -days 1825 \
  -in "$temp_dir/server.csr" \
  -signkey "$output_dir/server-key.pem" \
  -extfile "$temp_dir/server-ext.cnf" \
  -out "$output_dir/server-cert.pem"

server_fp="$(openssl x509 -in "$output_dir/server-cert.pem" -noout -fingerprint -sha256 | cut -d= -f2 | tr -d ':' | tr 'A-F' 'a-f')"
printf '%s\n' "$server_fp" >"$output_dir/server-fingerprint.txt"

chmod 600 "$output_dir/server-key.pem"
chmod 644 "$output_dir/server-cert.pem" "$output_dir/server-fingerprint.txt"

echo "Common DexFull WSS certificate generated in: $output_dir"
echo "SHA-256 fingerprint to pin in every Unity client:"
echo "$server_fp"
echo "Securely deploy server-cert.pem and server-key.pem to every robot."
echo "Do not place server-key.pem in source control or a public package."
