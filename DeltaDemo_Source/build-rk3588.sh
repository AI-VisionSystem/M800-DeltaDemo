#!/usr/bin/env sh
set -eu
source_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
case "$(uname -m)" in aarch64|arm64) ;; *) echo 'M800 requires ARM64'; exit 1;; esac
cmake -S "$source_dir" -B "$source_dir/build-rk3588" -DCMAKE_BUILD_TYPE=Release -DDEMO_WITH_SDK=ON
cmake --build "$source_dir/build-rk3588" --parallel 4
ctest --test-dir "$source_dir/build-rk3588" --output-on-failure
