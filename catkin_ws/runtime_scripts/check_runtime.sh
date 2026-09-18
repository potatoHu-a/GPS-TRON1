#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

package_path="$(rospack find tron_local_collision_safety)"
exec "$package_path/scripts/check_runtime.sh" "$@"
