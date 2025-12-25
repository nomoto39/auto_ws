# auto_ws: FAST_LIO and nav2

FAST_LIOとnav2を用いた事前地図なしの他距離自律移動を実装するパッケージです。
NUC (x86_64) と Jetson Orin (ARM) の両方で動作するように調整しています。

## 1. System Requirements
- **OS:** Ubuntu 22.04
- **ROS 2:** Humble
- **Hardware:** - Livox Mid-360
  - Intel NUC (Core i7) or NVIDIA Jetson Orin Nano

## 2. Installation

### 依存ライブラリのインストール
```bash
sudo apt update
rosdep update
# 必要な依存関係を自動インストール
rosdep install --from-paths src --ignore-src -r -y
