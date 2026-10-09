# iq_taxi

## 默认编译（Linux 原生驱动和 GUI）

在 IQTAXI 源码根目录执行：

```bash
mkdir build && cd build
cmake ../
make
```

默认编译原生核心库、设备驱动、GUI、示例和工具，所有第三方应用插件均默认关闭，
不会查找 SoapySDR、UHD、GNU Radio、SDR++ 或 SDRangel 的依赖。
GUI 本身仍需要 OpenGL、GLFW3、GLEW 和 pkg-config 等基础开发依赖。

## 手动启用插件

安装对应应用的开发依赖后，在 `build` 目录重新运行 CMake，按需启用插件：

```bash
# 示例：只启用 UHD 插件
cmake ../ -DENABLE_UHD_IQTAXI=ON
make
```

各开关互相独立，均默认为 `OFF`，可组合传入多个 `-D...=ON`：

| 插件 | CMake 开关 | 主要依赖 / 配置 |
|------|------------|------------------|
| SoapySDR | `ENABLE_SOAPYSDR_IQTAXI` | SoapySDR 开发包 |
| UHD | `ENABLE_UHD_IQTAXI` | UHD、Boost 开发包 |
| GNU Radio | `ENABLE_GNURADIO_IQTAXI` | GNU Radio 开发包及对应 Python 绑定工具 |
| SDR++ | `ENABLE_SDRPP_IQTAXI` | `libsdrpp_core`，`-DSDRPP_ROOT=/path/to/SDRPlusPlus` |
| SDRangel | `ENABLE_SDRANGEL_IQTAXI` | Qt5、SDRangel 库，`-DSDRANGEL_SOURCE_DIR=/path/to/sdrangel` |
| HDSDR ExtIO | `ENABLE_HDSDR_IQTAXI` | Windows/MXE 构建，同时设置 `-DENABLE_WIN=ON` |

关闭已启用的插件时，将对应开关设为 `OFF`。CMake 会保留已有构建目录中的缓存值；
升级前已经配置过的目录不会自动关闭旧开关，可使用新的构建目录，或显式指定：

```bash
cmake ../ \
    -DENABLE_SOAPYSDR_IQTAXI=OFF \
    -DENABLE_UHD_IQTAXI=OFF \
    -DENABLE_GNURADIO_IQTAXI=OFF \
    -DENABLE_SDRPP_IQTAXI=OFF \
    -DENABLE_SDRANGEL_IQTAXI=OFF \
    -DENABLE_HDSDR_IQTAXI=OFF
```

## 安装 Linux 驱动和可选 UHD 插件

默认构建后可用 `sudo make install` 安装原生驱动和工具。
若需要 UHD 集成，从源码根目录执行以下命令，会安装一套相互匹配的
`libsdr_core`、`libsdr_driver` 和 UHD `libIQTaxiUHD` 插件：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_UHD_IQTAXI=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/local
make -C build -j"$(nproc)"
sudo make -C build install
```

启用 UHD 插件后，`sudo make install` 还会检查三者是否都已安装并自动执行
`ldconfig`。不要只复制 UHD 插件，否则插件与运行库版本不一致时，
GNU Radio/gr-osmosdr 可能在创建 RX streamer 时崩溃。已有构建目录若曾
使用其他安装前缀，请先重新运行上面的 `cmake -S ...` 配置命令。

## E100 sample rates

E100 上位机驱动与当前 legacy-rate 下位机一致，支持以下 10 档采样率：

`1.92 / 3.84 / 5.76 / 7.68 / 11.52 / 15.36 / 23.04 / 30.72 / 61.44 / 122.88 MSPS`

`E100Impl::setSampleRate()` 会像 E206 一样将请求量化到最近的支持档位。公共定义在
`include/sdr/api/SampleRates.hpp`，E100/E206 核心驱动、E100 GUI、SoapySDR、
SDR++ 和 SDRangel 都从这里取得 MP2021 采样率能力表。

## 参考时钟切换

E100/E200/E206 的 `set_vcxo_reference_source()` 完成下发命令、检查 ACK、
轮询 VCXO 状态后才返回：外部 10 MHz 要求参考有效、检测到 10 MHz 且已锁定；
PPS 要求参考有效、检测到 PPS 且已锁定；手动 DAC 模式只确认模式已选择。
所有模式均检查回读的时钟源与请求一致，默认最多等待 15 秒，每 500 ms 询问一次状态。
原生 `Device::set_clock_reference_source()` 和 UHD `set_clock_source()` 均调用这套驱动流程。
原生接口保留单参数调用，并提供 `(source, timeout_seconds)` 重载；
UHD 可通过设备参数 `clock_lock_timeout=30` 调整等待时间（单位秒，必须为有限正数）。
未锁定或状态读取失败会抛出异常；锁定超时信息包含最后读取的状态值。
命令已确认但等待失败时不会自动切回原时钟源，UHD `get_clock_source()` 返回本次下发的源，
调用者可查询 `ref_locked` 或显式切回内部模式。

`set_clock_source("internal")` 保留当前 DAC 值并切换到手动 DAC 自由运行模式，
等待确认模式已切换；该模式不要求外部参考锁定。
`ref_locked` 对外部源使用上述锁定条件，对内部源检查是否已选择手动 DAC 模式。
E100 固件需支持独立的 VCXO 命令 `0x003f/0x0040` 和状态回读 `0x0021`，
初始化时默认选择内部模式也会使用这些命令。

## E200 examples

E200 上位机示例程序使用方法见：

- [E200_EXAMPLES_USAGE.md](E200_EXAMPLES_USAGE.md)
