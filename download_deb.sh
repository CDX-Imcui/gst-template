#!/bin/bash

# 确保脚本出错就退出
set -e

# 目标目录
TARGET_DIR="/deb"
if [ ! -d "$TARGET_DIR" ]; then
    mkdir -p "$TARGET_DIR"
fi
cd "$TARGET_DIR"

# 定义函数：下载包和依赖
download_pkg() {
    local pkg="$1"
    echo ">>> 处理包: $pkg"
    rm -rf "$pkg"   # 删除旧目录，重新下载
    mkdir -p "$pkg"
    cd "$pkg"

    apt-get download $(
        apt-cache depends --recurse --no-recommends \
                          --no-suggests --no-conflicts \
                          --no-breaks --no-replaces \
                          --no-enhances "$pkg" \
        | grep "^\w" | sort -u
    )
    dpkg-scanpackages . /dev/null | gzip -9c > Packages.gz
    cd ..
}



sudo apt install dpkg-dev # 每次下载完毕立即索引
packages=(libopencv-dev
              build-essential
              cmake
              git
              pkg-config
              meson
              ninja-build
              )
# 找到最后一个存在目录的下标
last_index=-1
for i in "${!packages[@]}"; do
    if [ -d "${packages[$i]}" ]; then
        last_index=$i
    fi
done
echo ">>> 从${packages[$last_index]}开始下载"
# 按顺序下载
for ((i=$last_index; i<${#packages[@]}; i++)); do
    download_pkg "${packages[$i]}"
done


sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
echo ">>> 换阿里云源，为了gstream1.20"
sudo tee /etc/apt/sources.list > /dev/null <<EOF
deb http://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse
EOF

sudo apt update

packages_aliyun=(
              libgstreamer1.0-dev
              libgstreamer-plugins-base1.0-dev
              gstreamer1.0-rtsp
              libgstreamer-plugins-bad1.0-dev
              gstreamer1.0-plugins-base
              gstreamer1.0-plugins-good
              gstreamer1.0-plugins-bad
              gstreamer1.0-plugins-ugly
              gstreamer1.0-libav
              gstreamer1.0-tools
              gstreamer1.0-x
              gstreamer1.0-alsa
              gstreamer1.0-gl
              gstreamer1.0-gtk3
              gstreamer1.0-qt5
              gstreamer1.0-pulseaudio
              )
# 找到最后一个存在目录的下标
last_index=-1
for i in "${!packages_aliyun[@]}"; do
    if [ -d "${packages_aliyun[$i]}" ]; then
        last_index=$i
    fi
done
echo ">>> 从${packages_aliyun[$last_index]}开始下载"
# 按顺序下载
for ((i=$last_index; i<${#packages_aliyun[@]}; i++)); do
    download_pkg "${packages_aliyun[$i]}"
done

#for pkg in "${packages_aliyun[@]}"; do
#    download_pkg "$pkg"
#done

if [ -f /etc/apt/sources.list.backup ]; then
    echo ">>> 恢复原始源 sources.list"
    sudo mv /etc/apt/sources.list.backup /etc/apt/sources.list
    sudo apt update
fi

echo ">>> 全部完成"