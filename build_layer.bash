#!/bin/bash
# 按层及其依赖编译，用法: ./build_layer.bash c|p|e|a [--dry-run]
set -euo pipefail
cd "$(dirname "$(realpath "$0")")"

dry_run=false
layer=""
for arg in "$@"; do
  case "$arg" in
    --dry-run) dry_run=true ;;
    c|communication) selected=communication ;;
    p|perception) selected=perception ;;
    e|execution) selected=execution ;;
    a|application) selected=application ;;
    *) echo "用法: $0 {c|p|e|a|communication|perception|execution|application} [--dry-run]" >&2; exit 1 ;;
  esac
  if [[ "$arg" != --dry-run ]]; then
    [[ -z "$layer" ]] || { echo "只能选择一个层" >&2; exit 1; }
    layer="$selected"
  fi
done
[[ -n "$layer" ]] || { echo "请指定构建层 c|p|e|a" >&2; exit 1; }

# colcon list 是包名的唯一来源；路径仅用于判断包属于哪个层。
listing=$(colcon list --base-paths src)
packages=()
while IFS=$'\t' read -r name path kind; do
  [[ -n "$name" && -n "$path" ]] || continue
  package_path=$(realpath "$path")
  case "$package_path" in
    "$PWD/src/$layer/"*) packages+=("$name") ;;
  esac
done <<< "$listing"
(( ${#packages[@]} > 0 )) || { echo "层 $layer 未发现可构建包" >&2; exit 1; }
command=(colcon build --base-paths src --build-base build --install-base install --symlink-install --packages-up-to "${packages[@]}")
if "$dry_run"; then
  printf '层包选择集合（colcon 将补齐依赖闭包）:\n'
  printf '  %s\n' "${packages[@]}"
  printf '命令:'
  printf ' %q' "${command[@]}"
  printf '\n'
else
  "${command[@]}"
  echo "=== 编译完成 ==="
fi
