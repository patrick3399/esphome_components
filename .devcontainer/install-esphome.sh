#!/usr/bin/env bash
set -euo pipefail

python -m pip install --upgrade pip
mkdir -p "${HOME}/.esphome-language-server"

case "${ESPHOME_CHANNEL:-stable}" in
  stable)
    python -m pip install --upgrade esphome
    ;;
  beta)
    python -m pip install --upgrade --pre esphome
    ;;
  *)
    echo "Unsupported ESPHOME_CHANNEL: ${ESPHOME_CHANNEL}" >&2
    echo "Use 'stable' or 'beta'." >&2
    exit 1
    ;;
esac

esphome version
