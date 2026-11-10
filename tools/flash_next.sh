#!/bin/bash
# flash_next.sh - Convenience wrapper to flash the next sequential TCU
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/flash_tcu.py" "$@"

