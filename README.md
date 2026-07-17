# 4G 远程遥控车 — Luckfox Pico Plus

基于 Luckfox Pico（RV1103）的 4G 远程遥控车方案，支持 H.265 视频流推送、双向语音对讲、PWM 电机/舵机控制，通过反向代理实现跨网络远程控制。

**硬件开源地址**：<https://oshwhub.com/qwiaoei/project_jzgihors>

---

## 功能特性

- **视频**：SC3336 传感器 + USB 摄像头双路 H.265 编码，TCP 推流
- **控制**：TCP 服务端（端口 5103），PWM 电机驱动（RZ7899-MS）、舵机转向、灯光控制
- **通信**：ML307C 4G 模组拨号上网，rproxyc/rproxys 反向代理穿透 NAT
- **对讲**：ALSA 音频采集，G.711a 编码，双向语音
- **遥测**：每秒上报电池电压，支持倒车限速、非线性油门曲线

---

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    云端 / VPS                            │
│                     rproxys                              │
│          反向代理服务端，管理端口池，分配7个流端口          │
└──────────────────────┬──────────────────────────────────┘
                       │ TCP / UDP
        ┌──────────────┴──────────────┐
        │         4G / 互联网          │
        └──────────────┬──────────────┘
                       │
┌──────────────────────┴──────────────────────────────────┐
│              Luckfox Pico Plus（RV1103）                  │
│                                                         │
│  ┌─────────────┐  ┌──────────┐  ┌───────────────────┐  │
│  │  rkipc      │  │ rproxyc  │  │  remote_control   │  │
│  │  视频采集    │  │ 反向代理  │  │  TCP:5103 控制服务 │  │
│  │  H.265 编码  │  │ 客户端   │  │  PWM 电机/舵机/灯  │  │
│  │  TCP 推流    │  │ 7路流上传 │  │  ML307C 4G 拨号   │  │
│  └─────────────┘  └──────────┘  │  音频采集 G.711a   │  │
│                                  │  电池电压遥测      │  │
│                                  └───────────────────┘  │
│                                                         │
│  硬件接口：                                              │
│  · SC3336 CSI 摄像头  · USB 摄像头（UVC）               │
│  · ML307C 4G 模组（USB）  · RZ7899-MS 电机驱动（PWM8/9）│
│  · 转向舵机（PWM10）  · 灯光（PWM11）  · ADC 电池电压    │
└─────────────────────────────────────────────────────────┘
```

### 核心模块说明

| 模块 | 路径 | 说明 |
|------|------|------|
| `remote_control` | `project/app/remote_control/` | 主控制服务，TCP 服务端，PWM 输出，4G 拨号，音频采集 |
| `rkipc` | `project/app/rkipc/` | Rockchip IPC 视频采集，H.265 双码流编码，TCP 推流 |
| `rproxyc` | `project/app/rproxyc/` | 反向代理客户端，连接 rproxys 并上传 7 路流 |
| `rproxys` | `tools/rproxys/` | 反向代理服务端，部署在云端，管理设备连接和端口分配 |

---

## 目录结构

```
RC-luckfox/
├── project/
│   ├── build.sh                    # 主构建脚本
│   ├── rkflash.sh                  # 烧录脚本
│   ├── app/
│   │   ├── remote_control/         # 遥控车主程序
│   │   │   └── source/
│   │   │       ├── remote_control.c  # TCP 服务、PWM 控制、主循环
│   │   │       ├── ml307c.c          # ML307C 4G 模组 AT 指令、拨号、GPS
│   │   │       ├── audio.c           # ALSA 音频后端
│   │   │       └── log.c             # 日志模块
│   │   ├── rkipc/                  # 视频采集（基于 Rockchip media）
│   │   └── rproxyc/                # 反向代理客户端
│   └── cfg/
│       └── BoardConfig_IPC/        # 板级配置、overlay 开机脚本
├── tools/
│   ├── rproxys/                    # 反向代理服务端（x86 部署）
│   └── linux/                      # Linux 工具（烧录工具等）
├── media/                          # Rockchip media 库及算法
├── sysdrv/                         # U-Boot、Kernel、Buildroot
├── output/                         # 编译输出目录
└── IMAGE/                          # 打包好的固件镜像
```

---

## 编译

### 环境要求

- **操作系统**：Ubuntu 22.04（推荐）
- **依赖包**：

```bash
sudo apt-get install repo git ssh make gcc gcc-multilib g++-multilib \
  module-assistant expect g++ gawk texinfo libssl-dev bison flex \
  fakeroot cmake unzip gperf autoconf device-tree-compiler \
  libncurses5-dev pkg-config bc python-is-python3 passwd openssl \
  openssh-server openssh-client vim file cpio rsync
```

### 获取源码

```bash
git clone <本仓库地址> RC-luckfox
cd RC-luckfox
```

### 配置工具链

```bash
cd tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/
source env_install_toolchain.sh
cd ../../../..
```

### 编译全部固件

```bash
# 选择板级配置（Luckfox Pico Plus）
./build.sh lunch

# 编译全部（U-Boot + Kernel + Rootfs + App + 打包）
./build.sh allsave
```

### 仅编译应用层

```bash
# 编译所有 app（remote_control、rkipc、rproxyc 等）
./build.sh app

# 编译完成后打包 firmware
./build.sh firmware
```

### 编译 rproxys（服务端，x86）

```bash
cd tools/rproxys
make
```

### 编译输出

编译完成后，固件输出到 `output/image/`：

```
output/image/
├── update.img      # 完整升级包（推荐用于烧录）
├── boot.img
├── rootfs.img
├── oem.img
├── uboot.img
├── trust.img
├── parameter.txt
└── download.bin
```

---

## 烧录

使用 **SocToolKit** 烧录固件：

1. 将 Luckfox Pico Plus 通过 USB 连接电脑，进入 Maskrom 模式
2. 打开 SocToolKit，选择 `output/image/` 目录下的固件镜像
3. 点击烧录，等待完成即可

---

## 部署与运行

### 设备端自动启动

系统固件已配置 `remote_control` 开机自启动（通过 init 脚本 `S98remote_control`）：

```
/oem/usr/bin/remote_control   # 主程序路径
```

启动顺序：
1. `rkipc` 视频服务启动（由系统 init 启动）
2. `remote_control` 启动，初始化 PWM、TCP 服务端
3. ML307C 模组拨号获取 IP
4. 获取 IMEI 后自动启动 `rproxyc` 连接反向代理服务器

### 服务端部署（rproxys）

将 `tools/rproxys/proxy_server.tar.gz` 上传至云端 VPS（Ubuntu 24.04），解压后执行安装脚本：

```bash
tar xzf proxy_server.tar.gz
cd proxy_server
sudo ./install.sh
```

安装脚本自动完成以下操作：
- 二进制文件安装至 `/usr/bin/rproxys`
- 配置文件安装至 `/etc/rproxys.conf`（已有配置不会被覆盖）
- 注册 systemd 服务，开机自启动

安装完成后管理服务：

```bash
systemctl {start|stop|restart|status} rproxys
```

`rproxys` 会为每个接入设备分配 7 个端口：

| 端口用途 | 说明 |
|---------|------|
| video0 | 主码流视频（H.265） |
| video1 | 子码流视频（H.265） |
| audio | 双向语音对讲 |
| control | TCP/UDP 控制指令 |
| videoCtrl | USB 摄像头视频 |
| ssh | SSH 透传 |
| rearCam | 后置摄像头 |

### 客户端连接

APP 端通过 rproxys 分配的端口连接设备，实现视频观看、车辆控制、语音对讲。

---

## 硬件说明

本项目基于 Luckfox Pico Plus（RV1103）作为主控，搭配自研载板扩展以下硬件：

| 硬件 | 型号/接口 | 说明 |
|------|----------|------|
| 主控 | RV1103（Luckfox Pico Plus） | ARM Cortex-A7, 64MB DDR2, SPI NAND |
| 前摄像头 | SC3336（CSI） | 300W 像素，H.265 编码 |
| 后摄像头 | USB 摄像头（UVC） | 可选，720P |
| 4G 模组 | ML307C（UART） | 拨号上网、GPS、AT 指令控制 |
| 电机驱动 | RZ7899-MS（PWM8/9） | 内置电调模式，支持正反转、倒车限速 |
| 转向舵机 | 标准 PWM 舵机（PWM10） | 50Hz，500-2500us 脉宽 |
| 灯光 | MOS 管控制（PWM11） | 亮度可调 |
| 电池检测 | ADC 分压采集 | 每秒上报电压 |

**载板硬件开源**：<https://oshwhub.com/qwiaoei/project_jzgihors>

---

## 配置说明

### 视频配置（rkipc-300w.ini）

关键配置项：

```ini
[video.source]
enable_tcp_video = 1        # 启用 TCP 视频推流
enable_npu = 0              # 禁用 AI（节省内存）

[video.0]                   # 主码流
width = 2304
height = 1296

[video.1]                   # 子码流
width = 1280
height = 720
```

### 控制参数（remote_control）

| 参数 | 值 | 说明 |
|------|----|------|
| TCP 端口 | 5103 | 控制指令接收端口 |
| 电机 PWM 频率 | 1000Hz | 内置电调模式 |
| 前进最大占空比 | 512 | 全速前进 |
| 倒车最大占空比 | 150 | 限速保护 |
| 舵机脉宽范围 | 500-2500us | 标准 50Hz 舵机信号 |
| 油门曲线 | 二次方 S 曲线 | 精细化低速控制 |

---

## 许可证

本项目基于 Luckfox Pico SDK 修改，遵循原 SDK 许可协议。  
硬件设计开源，详见 [嘉立创开源广场](https://oshwhub.com/qwiaoei/project_jzgihors)。
