#!/usr/bin/env bash
# Build and run the host-side unit tests.
#
# `pio test -e native` is the intended entry point, but this Windows machine has no
# host compiler -- only the cross toolchains PlatformIO installed for the ESP32.
# WSL does have g++, and PlatformIO has already vendored Unity into
# .pio/libdeps/native/Unity, so the tests can be compiled directly with no new
# installs on either side.
#
# 中文说明（为什么不用 pio test 而是这个脚本）：
#   本来应该直接 `pio test -e native`，但这台 Windows 上没有本机编译器 ——
#   PlatformIO 只装了给 ESP32 用的交叉工具链，编不出能在电脑上跑的程序。
#   而 WSL 里有 g++，Unity 的源码 PlatformIO 也已经拉到 .pio/libdeps 下了，
#   所以直接用 g++ 编译就行，两边都不需要额外安装任何东西。
#   换到有 gcc 的机器上时，`pio test -e native` 依然可用，测试代码本身没有改动。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNITY="$ROOT/.pio/libdeps/native/Unity/src"
OUT="$ROOT/.pio/native_tests"

if [[ ! -f "$UNITY/unity.c" ]]; then
  echo "Unity source not found at $UNITY"
  echo "Run 'pio pkg install -e native' once to vendor it (needs no host compiler), then retry."
  exit 1
fi

mkdir -p "$OUT"

INCLUDES=(-I"$UNITY")
LIB_SRC=()
for d in "$ROOT"/lib/*/; do
  INCLUDES+=(-I"$d")
  for f in "$d"*.cpp; do
    [[ -e "$f" ]] && LIB_SRC+=("$f")
  done
done

fail=0
total=0
for testdir in "$ROOT"/test/test_*/; do
  name="$(basename "$testdir")"
  src=("$testdir"*.cpp)
  [[ -e "${src[0]}" ]] || continue
  total=$((total + 1))

  if ! g++ -std=c++17 -Wall -Wextra -O1 \
        "${INCLUDES[@]}" \
        "${src[@]}" "${LIB_SRC[@]}" "$UNITY/unity.c" \
        -lm -o "$OUT/$name"; then
    echo "=== $name: COMPILE FAILED ==="
    fail=$((fail + 1))
    continue
  fi

  echo "=== $name ==="
  if ! "$OUT/$name"; then
    fail=$((fail + 1))
  fi
  echo
done

echo "================================================"
if [[ $fail -eq 0 ]]; then
  echo "ALL $total TEST SUITES PASSED"
else
  echo "$fail of $total TEST SUITES FAILED"
fi
exit $fail
