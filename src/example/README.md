# IQTAXI 示例目录

示例程序按设备和用途分目录组织：

| 目录 | 内容 |
| --- | --- |
| `e100/` | E100 控制、收发、录制、回放和 GUI 示例 |
| `e200/` | E200 IQ 收发、双通道 RX、录制和回放示例 |
| `e206/` | E206 IQ 收发和录制示例 |
| `m300/` | M300 PCIe/XDMA、AD9361、RX/TX probe 示例 |
| `tools/` | 跨设备工具，例如固件更新 GUI |
| `third_party/` | 示例 GUI 使用的第三方源码 |

构建出来的可执行文件名保持不变，例如：

```bash
cmake --build build --target m300_rx_iq_probe m300_tx_iq_probe
```

M300 当前原生 IQTAXI 主线是 RX-first：

- `m300_iqtaxi_rx_example` 展示最终交付 API 风格，只使用公共 `Device` 和 `rx_streamer` 接口。
- RX 通过 `M300_XDMA` 和 `/dev/xdma0_c2h_1` 接入。
- 控制面通过 `/dev/xdma0_h2c_0` 和 `/dev/xdma0_c2h_0` 接入。
- TX probe 可用于 FPGA bring-up，但正式 `m300_tx_streamer` 还未接入统一 IQTAXI API。

M300 在线烧录地址 0 的单一启动镜像，并回读校验：

```bash
# 在 xilinx_image_builder 仓库根目录执行；默认会要求输入 PROGRAM 确认。
host_app/e200/MICROPHASE_IQ_TAXI/build/src/example/m300_flash_update \
  --bin hdl/m300/vivado/project/m300_golden/m300_golden.runs/impl_1/system_top.bin \
  --base /dev/xdma0
```

文件长度无需预先对齐，程序会在内存中以 `0xFF` 补齐到 4 KiB 后进行
XDMA 传输，固定从 Flash `0x00000000` 开始擦写。写完后通过
`/dev/xdma0_c2h_1` 回读输入文件的原始长度，默认保存为
`system_top.bin.readback.bin`，并逐字节校验；只有校验成功才从地址 0
重载。可用 `--readback PATH` 指定回读文件，自动化调用可加 `--yes`，调试
时可加 `--no-reboot`。如果镜像已经写入，只需回读现有 Flash，可加
`--verify-only`；该模式不会擦除、写入或重启。单镜像没有掉电回退，擦写
期间不要断电。
