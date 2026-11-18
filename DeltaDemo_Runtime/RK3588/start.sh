#!/bin/sh
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
export LD_LIBRARY_PATH="$here/lib:$here${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QT_PLUGIN_PATH="$here/plugins"
export QT_QPA_PLATFORM_PLUGIN_PATH="$here/plugins/platforms"
# Windows file copying can discard executable permissions.
test -x "$here/DeltaDemo" || chmod u+x "$here/DeltaDemo"
exec "$here/DeltaDemo" "$@"
