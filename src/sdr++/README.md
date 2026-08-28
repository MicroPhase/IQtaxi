# SDR++ IQTAXI Plugin

树外 SDR++ source 模块：让 SDR++ **直接**使用 IQTAXI 原生驱动（`Device` / `rx_streamer`），不经过 UHD/Soapy。

一个 `.so`（`iqtaxi_source.so`）在 Source 列表中显示为 **IQTAXI**，面板里再选板卡：

| Device 下拉 | `makeDevice` | RX 增益 | 采样率 |
|-------------|--------------|---------|--------|
| E100 | `E100` | 0～41 dB | 11 档：15.36 MHz 族（含 1.92）+ 46.08 MHz 族。6G/10G 由固件读回 |
| E200 | `E200` | 0～75 dB | AD9361 常用档，最高 61.44 MHz |
| E206 | `E206` | 0～41 dB | 固件 22 档 |

父工程默认打开本插件（与 `ref/E100` 相同），源码路径默认 `/home/kang/packages/SDRPlusPlus`。找不到 `libsdrpp_core` 或头文件时会 skip。

## 构建

```bash
cmake ..
make -j"$(nproc)" iqtaxi_source
sudo cmake --install .
```

也可显式指定：

```bash
cmake .. \
  -DENABLE_SDRPP_IQTAXI=ON \
  -DSDRPP_ROOT=/path/to/SDRPlusPlus
```

依赖：

- 本仓库的 `sdr_core` / `sdr_driver`（父工程一起编时直接链 CMake target，不必先 `make install`）
- 已安装的 SDR++（`libsdrpp_core`）
- SDR++ 源码头文件（`core/src/module.h`）

### 选项

| 变量 | 默认 | 说明 |
|------|------|------|
| `ENABLE_SDRPP_IQTAXI` | ON | 父工程是否尝试编译本插件 |
| `SDRPP_ROOT` | `/home/kang/packages/SDRPlusPlus` | SDR++ 源码树 |
| `SDRPP_PLUGIN_INSTALL_DIR` | `lib/sdrpp/plugins` | 相对 `CMAKE_INSTALL_PREFIX` 的安装目录 |
| `IQTAXI_LIB_DIR` | `/usr/local/lib` | **仅独立编本目录时** 查找 `libsdr_core.so` / `libsdr_driver.so` |

旧 CMake 选项名 `ENABLE_SDRPP_IQTAXI_E206` 已废弃，请改用 `ENABLE_SDRPP_IQTAXI`。

### 示例

```bash
cd /path/to/MICROPHASE_IQ_TAXI
mkdir -p build && cd build
cmake .. \
  -DENABLE_SDRPP_IQTAXI=ON \
  -DSDRPP_ROOT=/path/to/SDRPlusPlus
make -j"$(nproc)" iqtaxi_source
sudo cmake --install .
```

产物（默认 prefix `/usr/local`）：

```text
/usr/local/lib/sdrpp/plugins/iqtaxi_source.so
```

## 注册到 SDR++

SDR++ 不会自动加载树外插件，需要写入用户配置：

```bash
./src/sdr++/register_sdrpp_plugin.sh
# 或指定 .so：
./src/sdr++/register_sdrpp_plugin.sh /usr/local/lib/sdrpp/plugins/iqtaxi_source.so
```

脚本会改 `~/.config/sdrpp/config.json`（`modules` + `moduleInstances`）。完全退出再开 SDR++。

卸载登记：

```bash
./src/sdr++/register_sdrpp_plugin.sh --remove
```

## 使用

1. Source → 选择 **IQTAXI**
2. Device 选 E100 / E200 / E206
3. 填写 Host IP，选采样率 / 增益，Start

插件运行参数另存为 `~/.config/sdrpp/iqtaxi_source_config.json`（IP、型号、采样率、增益）。

## 源码说明

- `main.cpp`：SDR++ 模块入口、UI、收数线程
- `register_sdrpp_plugin.sh`：把插件登记进 SDR++ `config.json`
- `CMakeLists.txt`：可随父工程编，也可在本目录独立编
