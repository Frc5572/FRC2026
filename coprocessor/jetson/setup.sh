#!/usr/bin/env bash

set -euo pipefail

IFS=$'\n\t'
readonly SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

SOURCE_FILE="coprocessor.service"
REMOTE_USER="orin"
REMOTE_HOST="orin"
DEST_PATH="/etc/systemd/system/"

if scp "$SOURCE_FILE" "$REMOTE_USER@$REMOTE_HOST:$DEST_PATH"; then
    echo "Transfer successful"
else
    echo "Transfer failed"
    exit 1
fi

if ssh $REMOTE_USER@$REMOTE_HOST 'bash -s' < enable-service.sh; then
    echo "Setup Successful"
else
    echo "Setup Failed"
    exit 1
fi