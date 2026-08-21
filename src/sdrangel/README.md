# SDRangel IQTAXI Plugin

树外 SDRangel 输入插件：让 SDRangel **直接**使用 IQTAXI 原生驱动（`Device` / `rx_streamer`），不经过 UHD/Soapy。

一个 `.so`（`libinputiqtaxi.so`）在设备列表中直接给出三种板卡：

| 设备列表名称 | hardwareId | `makeDevice` |
|--------------|------------|--------------|
| IQTAXI E100 | `IQTAXI-E100` | `E100` |
| IQTAXI E200 | `IQTAXI-E200` | `E200` |
| IQTAXI E206 | `IQTAXI-E206` | `E206` |

采样率 / 增益范围随所选设备变化。

## 构建

父工程 `cmake ..` 时打开：

```bash
cmake .. \
  -DENABLE_SDRANGEL_IQTAXI=ON \
  -DSDRANGEL_SOURCE_DIR=/path/to/sdrangel
```

依赖：

- 本仓库的 `sdr_core` / `sdr_driver`
- 系统或自编译的 SDRangel（`libsdrbase` / `libsdrgui`）
- Qt5 Core/Widgets
- SDRangel 源码头文件（`plugininterface.h` 等）

### 选项

| 变量 | 默认 | 说明 |
|------|------|------|
| `ENABLE_SDRANGEL_IQTAXI` | OFF | 父工程是否 `add_subdirectory(src/sdrangel)` |
| `BUILD_SDRANGEL_IQTAXI_PLUGIN` | ON | 是否生成目标 `iqtaxiinput` |
| `BUILD_SDRANGEL_IQTAXI_SKELETON` | ON | 是否编静态后端骨架 |
| `SDRANGEL_SOURCE_DIR` | 空（也可使用同名环境变量） | SDRangel 源码树 |
| `SDRANGEL_BASE_LIB_DIR` | `/usr/local/lib/sdrangel` | `libsdrbase.so` 所在目录 |
| `SDRANGEL_PLUGIN_INSTALL_DIR` | `/usr/local/lib/sdrangel/plugins` | 插件安装目录 |

### 示例

```bash
cd /path/to/MICROPHASE_IQ_TAXI
mkdir -p build && cd build
cmake .. \
  -DENABLE_SDRANGEL_IQTAXI=ON \
  -DSDRANGEL_SOURCE_DIR=/path/to/sdrangel
make -j"$(nproc)" iqtaxiinput
sudo cmake --install . --component SDRangelIQTAXIPlugin
```

产物：

```text
/usr/local/lib/sdrangel/plugins/libinputiqtaxi.so
```

若机器上还有旧的 `libinputiqtaxie206.so`，请删掉以免冲突：

```bash
sudo rm -f /usr/local/lib/sdrangel/plugins/libinputiqtaxie206.so
```

## 使用

1. 完全退出并重启 SDRangel
2. 添加 Rx 设备，选择 **IQTAXI E100 / E200 / E206**
3. 填写板子 IP，选采样率 / 增益，Start

## 源码说明

- `iqtaxiplugin.*`：插件入口；枚举三种 OriginDevice
- `iqtaxiinput.*` / `iqtaxigui.*`：SDRangel DeviceSampleSource + GUI
- `iqtaxi_settings.*` / `iqtaxi_worker.*` / `iqtaxi_backend.*`：IQTAXI 后端；`device_model` 区分板卡
