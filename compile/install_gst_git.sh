#!/bin/bash

set -euo pipefail

shellname="$(basename "$0")"
script_dir="$(cd "$(dirname "$0")" && pwd)"

# 版本: 与当前目录内 tar 包文件名一致: <pkg>-<version>.tar.xz
gst_version="1.20.3"
orc_version="0.4.32"
gst_src="${HOME}/gst"
prefix="/usr/local"

# 包名顺序: 先 core 与 orc, 再各插件
need_pkgs=(
  orc
  gstreamer
  gst-plugins-base
  gst-plugins-good
  gst-plugins-bad
  gst-plugins-ugly
  gst-libav
)

jobs="$(nproc)"
(( jobs > 1 )) && jobs=$((jobs-1))

log(){ echo "[`date +%H:%M:%S`] $*"; }

require_tars() {
  local missing=0
  for p in "${need_pkgs[@]}"; do
    local ver
    if [[ "$p" == "orc" ]]; then
      ver="$orc_version"
    else
      ver="$gst_version"
    fi
    if ! ls "${script_dir}/${p}-${ver}.tar."* >/dev/null 2>&1; then
      echo "[error] 缺少 ${p}-${ver}.tar.(xz|gz) 于当前目录"
      missing=1
    fi
  done
  [[ $missing -eq 0 ]] || exit 1
}

extract_pkg() {
  local name="$1" ver="$2"
  local tarball
  tarball=$(ls "${script_dir}/${name}-${ver}.tar."* 2>/dev/null | head -n1)
  [[ -n "$tarball" ]] || { echo "[error] 未找到 ${name}-${ver} 压缩包"; exit 1; }
  mkdir -p "${gst_src}"
  cd "${gst_src}"
  rm -rf "${name}-${ver}"
  log "解压 ${name}-${ver}"
  tar xf "${tarball}"
  cd "${name}-${ver}"
}

build_one() {
  local name="$1" ver
  if [[ "$name" == "orc" ]]; then
    ver="$orc_version"
  else
    ver="$gst_version"
  fi
  extract_pkg "$name" "$ver"

  if [[ -f meson.build ]]; then
    rm -rf build
#    meson setup build --prefix="${prefix}"#cdx
    case "$name" in
          orc)
            meson setup build  --libdir=lib
            ;;
          gstreamer)
            meson setup build  --libdir=lib -Dintrospection=enabled
            ;;
          gst-plugins-base)
            meson setup build  --libdir=lib -Dintrospection=enabled
            ;;
          gst-plugins-good|gst-plugins-bad|gst-plugins-ugly|gst-libav)
            meson setup build  --libdir=lib
            # 若需强制某些特性(示例):
            #   gst-plugins-bad) 额外加: -Dx265=enabled  (依赖存在才会成功)
            ;;
          *)
            meson setup build  --libdir=lib
            ;;
        esac
    ninja -C build -j "${jobs}"
    sudo ninja -C build install
  elif [[ -f autogen.sh || -f configure ]]; then
    if [[ -f autogen.sh ]]; then
      ./autogen.sh --prefix="${prefix}"
    else
      ./configure --prefix="${prefix}"
    fi
    make -j "${jobs}"
    sudo make install
  else
    echo "[error] ${name}-${ver} 缺少 meson.build / configure"
    exit 1
  fi
}

uninstall_one() {
  local name="$1" ver
  if [[ "$name" == "orc" ]]; then
    ver="$orc_version"
  else
    ver="$gst_version"
  fi
  local dir="${gst_src}/${name}-${ver}"
  [[ -d "$dir" ]] || { log "跳过(不存在) $dir"; return 0; }
  cd "$dir"
  log "卸载 ${name}-${ver}"
  if [[ -f meson.build && -d build ]]; then
    sudo ninja -C build uninstall || true
  elif [[ -f Makefile ]]; then
    sudo make uninstall || true
  fi
}

usage() {
  cat <<EOF
用法: ./${shellname} install | uninstall
  将所需 tar 包放在脚本同目录(离线，不下载)。
  需要: ${need_pkgs[*]}
  例如: gstreamer-${gst_version}.tar.xz
EOF
}

main() {
  [[ $# -eq 1 ]] || { usage; exit 1; }
  case "$1" in
    install)
      require_tars
      for p in "${need_pkgs[@]}"; do
        build_one "$p"
      done
      ;;
    uninstall)
      # 逆序卸载
      for ((i=${#need_pkgs[@]}-1; i>=0; --i)); do
        uninstall_one "${need_pkgs[$i]}"
      done
      ;;
    *)
      usage; exit 1;;
  esac
  log "完成 $1"
}

main "$@"