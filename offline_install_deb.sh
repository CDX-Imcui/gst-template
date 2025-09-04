#!/bin/bash

# 确保脚本出错就退出
set -e


sudo mv /etc/apt/sources.list /etc/apt/sources.list.bak

packages=(libopencv-dev
              build-essential
              cmake
              git
              pkg-config
              meson
              ninja-build
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
for package in "${packages[@]}"; do
    echo "deb [trusted=yes] file:/deb/$package ./" >> /etc/apt/sources.list
    sudo apt-get update
    sudo apt-get install -y "$package"
    echo "<<< 已安装 $package"
done

#恢复
sudo mv /etc/apt/sources.list.bak /etc/apt/sources.list
sudo apt-get update
echo ">>> 全部完成"