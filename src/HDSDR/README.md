# HDSDR ExtIO_MicroPhase

Windows HDSDR 的 ExtIO 适配：一份 `ExtIO_MicroPhase.dll` 直接走 IQTAXI 原生驱动（`Device` / `rx_streamer`），不经过 UHD/Soapy。

HDSDR 界面里没有设备下拉，型号和 IP 用环境变量指定。默认 **E206**、`192.168.1.10`。

| `IQTAXI_EXTIO_DEVICE` | 采样率 | RX 增益（1 dB 步进） | 默认采样率 |
|-----------------------|--------|----------------------|------------|
| `E100` | 11 档：15.36 MHz 族（含 1.92）+ 46.08 MHz 族 | 0～41 | 15.36 Msps |
| `E200` | AD9361 常用档，最高 **61.44** Msps | 0～75 | 30.72 Msps |
| `E206` | 固件 22 档（含 80/64 族和 122.88） | 0～42 | 15.36 Msps |

E100 打开后会按固件 RF 范围区分 6G / 10G，写入 `%TEMP%\ExtIO_MicroPhase.log`。

HDSDR 是 **32 位**程序，必须用 MXE 的 `i686-w64-mingw32.static` 交叉编译，不能用 `x86_64-w64-mingw32`。

本机现有工具链在 `/home/kang/mxe`（`i686-w64-mingw32.static-gcc` / `i686-w64-mingw32.static-cmake` 已可用）。下面先写安装，已装好可跳到「构建」。

## 安装 MXE

官方仓库：[https://github.com/mxe/mxe](https://github.com/mxe/mxe)。只编 HDSDR 需要 **32 位静态**目标 + `gcc` + `cmake`。第一次编译 gcc 会比较久。

### 1. 主机依赖（Debian / Ubuntu）

```bash
sudo apt-get install \
    autoconf automake autopoint bash bison bzip2 flex g++ g++-multilib \
    gettext git gperf intltool libc6-dev-i386 libclang-dev \
    libgdk-pixbuf2.0-dev libltdl-dev libgl-dev libpcre2-dev libssl-dev \
    libtool-bin libxml-parser-perl lzip make openssl p7zip-full patch perl \
    python3 python3-mako python3-packaging python3-pkg-resources \
    python3-setuptools python-is-python3 ruby sed sqlite3 unzip wget xz-utils
```

完整列表以 [MXE Requirements](https://mxe.cc/#requirements-debian) 为准。

### 2. 克隆并编译工具链

建议装到 `$HOME/mxe`（当前环境即 `/home/kang/mxe`）：

```bash
git clone https://github.com/mxe/mxe.git "$HOME/mxe"
cd "$HOME/mxe"
make MXE_TARGETS='i686-w64-mingw32.static' gcc cmake -j"$(nproc)"
```

可选再编 `ccache`（MXE cmake 默认会走 ccache 包装器）。编完后应有：

```text
$HOME/mxe/usr/bin/i686-w64-mingw32.static-g++
$HOME/mxe/usr/bin/i686-w64-mingw32.static-cmake
```

### 3. 加入 PATH

写入 `~/.bashrc`（或当前 shell）：

```bash
export PATH="$HOME/mxe/usr/bin:$PATH"
```

检查：

```bash
which i686-w64-mingw32.static-cmake
i686-w64-mingw32.static-g++ --version
```

## 构建（Linux + MXE）

不要用系统 `/usr/bin/cmake` 直接加 toolchain file，否则会走到 ccache 包装器并报：

```text
ccache: error: Could not find compiler "i686-w64-mingw32.static-g++" in PATH
```

```bash
export PATH="$HOME/mxe/usr/bin:$PATH"

cd /path/to/IQtaxi
rm -rf build-win
mkdir build-win && cd build-win

i686-w64-mingw32.static-cmake .. -DENABLE_WIN=ON
make ExtIO_MicroPhase -j"$(nproc)"
```

`ENABLE_WIN=ON` 时只编 Windows 静态库、部分示例和本 ExtIO，不编 Soapy / UHD / GNU Radio。

产物：

```text
build-win/src/HDSDR/ExtIO_MicroPhase.dll
```

## 部署到 Windows

1. 安装 [HDSDR](https://www.hdsdr.de/)
2. 把 `ExtIO_MicroPhase.dll` 拷到 `HDSDR.exe` **同一目录**
3. 用下面的 bat 启动（不要直接双击 `HDSDR.exe`，否则仍是默认 E206）

在 HDSDR 目录新建 `start_hdsdr_e100.bat`：

```bat
@echo off
set IQTAXI_EXTIO_DEVICE=E100
set IQTAXI_EXTIO_ADDR=192.168.1.10
start "" "%~dp0HDSDR.exe"
```

E200 / E206 把 `IQTAXI_EXTIO_DEVICE` 改成对应名字即可。IP 按板卡实际地址改。

长期固定也可以在 Windows「环境变量」里新建同样两个用户变量，然后**重新打开** HDSDR。ExtIO 只在加载 DLL 的 `InitHW` 时读一次。

## 使用

1. 用 bat 启动 HDSDR
2. Options → Select Input → 选 **ExtIO_MicroPhase**（Hardware 显示 MicroPhase，Model 为 E100/E200/E206）
3. 选采样率 / 衰减（此处按 RX 增益写入驱动）
4. Start

## 日志

Windows：`%TEMP%\ExtIO_MicroPhase.log`  
例如 `C:\Users\<用户名>\AppData\Local\Temp\ExtIO_MicroPhase.log`

打开成功时会有 `OpenHW ok E100 6G` 或 `E100 10G`。

## 常见问题

- **系统 cmake 配置失败**：必须用 `i686-w64-mingw32.static-cmake`，并保证 `PATH` 含 `$HOME/mxe/usr/bin`。失败过的 `build-win` 先删掉再配。
- **HDSDR 找不到硬件**：DLL 文件名必须以 `ExtIO_` 开头，且与 `HDSDR.exe` 同目录。
- **连不上板**：确认 `IQTAXI_EXTIO_DEVICE` / `IQTAXI_EXTIO_ADDR` 在启动前已设置，改完后要重启 HDSDR。
- **E200 没有 122.88 Msps**：正常，E200 最高 61.44 Msps。
- **没有配置窗口**：`ShowGUI()` 为空，只能靠环境变量选设备和 IP。
