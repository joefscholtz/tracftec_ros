#!/usr/bin/env bash
set -euo pipefail

# Resolve script directory (absolute path)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Workspace root (two levels up from script dir)
WORKSPACE_ROOT="$(realpath "$SCRIPT_DIR/../..")"

MODULE_NAME="tracftec_ros"
MODULE_PATH="$SCRIPT_DIR"
OUTPUT_JUSTFILE="$WORKSPACE_ROOT/justfile"

# Get recipes from the module justfile
recipes=$(just --justfile "$MODULE_PATH/justfile" --list | tail -n +2 | awk '{print $1}')

{
  echo "# Auto-generated top-level justfile forwarding recipes to $MODULE_NAME module"
  echo
  echo "mod $MODULE_NAME 'src/$MODULE_NAME'"
  echo
  echo "default:"
  echo "    @just $MODULE_NAME"
  echo

  for recipe in $recipes; do
    if [[ "$recipe" == "default" ]]; then
      continue
    fi
    if [[ "$recipe" == "sync" ]]; then
      cat <<EOF
$recipe:
    @bash ./src/$MODULE_NAME/sync_justfile.sh

EOF
      continue
    fi
    cat <<EOF
$recipe:
    @just $MODULE_NAME $recipe

EOF
  done
} >"$OUTPUT_JUSTFILE"

echo "Generated $OUTPUT_JUSTFILE forwarding to $MODULE_NAME module recipes."
