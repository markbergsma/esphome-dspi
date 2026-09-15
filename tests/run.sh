#!/usr/bin/env bash
# Host-side tests. These need no hardware and no ESP toolchain.
set -euo pipefail

cd "$(dirname "$0")/.."

echo "== framing unit tests =="
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
c++ -std=c++17 -Wall -Wextra -Werror -Icomponents/dspi -o "$out/test_framing" tests/test_framing.cpp
"$out/test_framing"

echo
echo "== esphome config =="
# Validating both configs is the point: standalone.yaml contains no audio
# components at all, so it fails if the dspi component ever picks up a
# dependency on speaker, i2s_audio, media_player or sendspin.
#
# examples/control-only.yaml carries the github:// source users copy, so it is
# validated through a scratch copy pointing at this working tree instead --
# otherwise CI would be testing whatever is published rather than this commit.
# tests/standalone.yaml already uses a local path and needs no such treatment.
#
# Both spellings are rewritten: the published github:// source, and a relative
# local path. The scratch copy lives outside the repository, so a path relative
# to the example's own directory would not resolve there.
mkdir -p "$out/examples"
sed -e 's|source: github://markbergsma/esphome-dspi@main|source: {type: local, path: PLACEHOLDER}|' \
    -e 's|path: \.\./components|path: PLACEHOLDER|' \
  examples/control-only.yaml >"$out/examples/control-only.yaml"
sed -i.bak "s|PLACEHOLDER|$PWD/components|" "$out/examples/control-only.yaml"
cp examples/secrets.yaml "$out/examples/secrets.yaml" 2>/dev/null || cat >"$out/examples/secrets.yaml" <<'EOF'
wifi_ssid: "ci"
wifi_password: "ci-password"
EOF

for cfg in tests/standalone.yaml "$out/examples/control-only.yaml"; do
  echo "-- $cfg"
  esphome config "$cfg" >/dev/null
done

echo
echo "All host tests passed."
