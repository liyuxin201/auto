#!/usr/bin/env bash
set -euo pipefail

exec /usr/bin/codex -m gpt-5.5 "$@"
