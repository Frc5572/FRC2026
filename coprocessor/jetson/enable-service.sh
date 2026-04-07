#!/usr/bin/env bash

set -euo pipefail

IFS=$'\n\t'
readonly SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

sudo systemctl daemon-reload
sudo systemctl enable coprocessor
sudo systemctl start coprocessor