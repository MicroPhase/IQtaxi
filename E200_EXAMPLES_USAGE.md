# IQTAXI 上位机示例程序使用说明

本文说明 `host_app/e200/MICROPHASE_IQ_TAXI/src/example` 里 IQTAXI 相关示例程序的常用运行方法，覆盖单通道接收、单通道发送、单通道全双工、双通道接收、IQ 录制和 IQ 回放。当前 E206 可直接使用这些示例。

默认示例里的设备 IP 为 `192.168.1.10`，如果你的板卡地址不同，运行命令时用 `--addr` 改掉即可。

## 准备工作

主机网口需要配置到 E200 同一网段，例如主机使用 `192.168.1.100`：

```bash
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up
ping 192.168.1.10
```

E206 板端需要先启动 MP2021/GC080X 和控制服务：

```bash
/usr/sbin/e206_v25_server
```

`e206_v25_server` 会先做 legacy authorization 校验，通过后初始化 MP2021/GC080X，并启动兼容 IQTAXI 控制协议的 UDP 命令服务。

## 编译示例程序

在主机上编译 IQTAXI 上位机程序：

```bash
cd host_app/e200/MICROPHASE_IQ_TAXI
cmake -S . -B build -DENABLE_GNURADIO_IQTAXI=OFF
cmake --build build --target \
  e200_iq_example \
  e200_dual_rx_capture \
  e200_record_iq_test \
  e200_replay_iq_test \
  -j
```

编译完成后示例程序通常位于：

```bash
cd build/src/example
```

如果本机已经安装 GNU Radio 相关依赖，也可以不加 `-DENABLE_GNURADIO_IQTAXI=OFF`。当前 `src/example` 里还会编译 `e100_gui`，所以系统需要有 OpenGL、GLFW3、GLEW 这些开发库；如果 CMake 在 GUI 依赖处失败，先安装对应开发包再重新配置。

## GNU Radio IQTAXI

`src/gnuradio` 里提供 `IQTAXI RX Source` 和 `IQTAXI TX Sink` 两个 GRC block，`Device` 下拉框支持：

- `E100`
- `E200`
- `E206`
- `M300_XDMA`
- `FNIC_XDMA`

E206 可以直接打开示例工程：

```bash
gnuradio-companion src/gnuradio/e206_test.grc
```

或者直接运行生成好的 Python 示例：

```bash
python3 src/gnuradio/e206_test.py
```

## E200 VCXO / PLL 控制接口

E200 的 `E200Impl` 提供与 E206 一致的 VCXO 控制 API：

```cpp
#include "src/driver/E200/e200_impl.hpp"

E200Impl device("192.168.1.10");

// 选择 PPS 自动锁定。
device.set_vcxo_reference_source(E200Impl::VcxoReferenceSource::pps);

// 或选择外部 10 MHz 自动锁定。
device.set_vcxo_reference_source(E200Impl::VcxoReferenceSource::external_10mhz);

// 手动控制 LTC2630：先写入 16-bit DAC 值，再进入手动模式。
device.set_vcxo_manual_dac(42580u);
device.set_vcxo_reference_source(E200Impl::VcxoReferenceSource::manual_dac);

const E200Impl::VcxoStatus status = device.get_vcxo_status();
```

`VcxoStatus` 包含：

- `locked`：外部参考闭环已锁定；手动 DAC 模式下固定为 false
- `reference_valid`：当前外部参考有效
- `reference_is_10mhz`：检测到 10 MHz 参考
- `reference_is_pps`：检测到 PPS
- `selected_source`：当前为 PPS、10 MHz 或手动 DAC
- `dac_value`：LTC2630 当前 16-bit DAC 值
- `raw`：下位机返回的原始 32-bit 状态字

三个上位机调用对应的下位机命令分别为 `0x003d`、`0x003e` 和读回选择器
`0x0020`。选择 10 MHz 模式后，FPGA 会拉高 E200 的 `CLKIN_10MHz_REQ`
输出；切换到 PPS 或手动模式后该输出拉低。

## 数据格式

这些示例默认使用 `cs16` IQ 文件格式：

- 每个 IQ 样点为 `I:int16, Q:int16`
- 每个复数样点占 4 字节
- 文件为小端序原始二进制，没有文件头
- 单通道文件布局：`I0 Q0 I1 Q1 ...`
- 双通道接收示例会把两个通道拆成两个独立文件

## 单通道接收

使用 `e200_iq_example` 的 RX 模式。这个示例不会保存 IQ 文件，主要用于验证 RX 流是否持续出包、时间戳是否连续、主机是否丢包。

```bash
./e200_iq_example \
  --addr 192.168.1.10 \
  --mode rx \
  --duration 10 \
  --sample-rate 20000000 \
  --rx-lo 2400000000 \
  --rx-gain 20 \
  --rx-request 4096
```

运行结束后重点看：

- `samples`：实际收到的复数样点数
- `effective rate`：主机端有效接收速率
- `timestamp gaps`：软件侧看到的时间戳跳变
- `wire seq errors`：UDP/VITA 包序号错误
- `host drops`：主机接收队列丢包

## 单通道发送

使用 `e200_iq_example` 的 TX 模式发送一个本地产生的基带单音：

```bash
./e200_iq_example \
  --addr 192.168.1.10 \
  --mode tx \
  --duration 10 \
  --sample-rate 20000000 \
  --tx-lo 2400000000 \
  --tx-atten 10 \
  --tx-tone 1000000 \
  --tx-amp 0.10 \
  --tx-packet 1024
```

常用参数：

- `--tx-tone`：基带单音频率，单位 Hz
- `--tx-amp`：发送幅度，范围 `0..0.95`
- `--tx-packet`：每次发送的复数样点数
- `--tx-atten`：TX 衰减设置，示例程序按当前 driver 的索引/寄存器值传入

## 单通道全双工

同一个示例也支持 RX 和 TX 同时运行：

```bash
./e200_iq_example \
  --addr 192.168.1.10 \
  --mode duplex \
  --duration 10 \
  --sample-rate 20000000 \
  --rx-lo 2400000000 \
  --tx-lo 2400000000 \
  --rx-gain 20 \
  --tx-atten 10 \
  --tx-tone 1000000 \
  --tx-amp 0.10
```

这适合验证 TX 数据链路、RX 数据链路和射频环回。

## 双通道接收

使用 `e200_dual_rx_capture`，它会启用 RX 通道 0 和通道 1，并把两个通道分别保存为两个 `cs16` 文件：

```bash
./e200_dual_rx_capture \
  --addr 192.168.1.10 \
  --duration 10 \
  --sample-rate 20000000 \
  --rx-lo 2400000000 \
  --rx-gain 20 \
  --frames-per-read 4096 \
  --out0 ch0.cs16 \
  --out1 ch1.cs16
```

`--frames-per-read` 表示每次读取的双通道 sample frame 数。一个 frame 包含通道 0 的一个复数样点和通道 1 的一个复数样点。

输出文件：

- `ch0.cs16`：RX0 的 `I/Q` 数据
- `ch1.cs16`：RX1 的 `I/Q` 数据

## 双通道发送

当前 `src/example` 目录还没有独立的 E200 双通道 TX 示例程序。现有快速验证路径主要是：

- 单通道 TX：`e200_iq_example --mode tx`
- 单通道 RX/TX 全双工：`e200_iq_example --mode duplex`
- 双通道 RX：`e200_dual_rx_capture`

如果后续要补双通道 TX 示例，建议基于 `e200_iq_example.cpp` 扩展：

- 配置 `set_channel_enable(0x3)`
- 创建两个发送 buffer
- 调用 TX streamer 的多 buffer 发送接口
- 同时配置 TX1/TX2 的 LO、atten 和数据源

需要注意当前 FPGA 顶层 DAC0 已经支持 UOE TX 和 replay mux；DAC1 路径是否完整接入上位机 TX 链路，需要结合当前 HDL 版本确认后再开放双通道 TX 示例。

## IQ 录制

`e200_record_iq_test` 使用 FPGA 侧 DDR-backed record buffer 录制一段 RX0 IQ，然后通过 UDP 读回到主机文件。

录制 16 MiB：

```bash
./e200_record_iq_test \
  --addr 192.168.1.10 \
  --output e200_record.cs16 \
  --length-mb 16 \
  --sample-rate 20000000 \
  --rx-lo 2400000000 \
  --rx-gain 20
```

常用参数：

- `--length-mb`：录制长度，单位 MiB；当前要求 `4..256 MiB` 且是 4 MiB 的整数倍
- `--bytes`：直接按字节指定录制长度，也要求 4 MiB 对齐
- `--block`：DMA 读回分块大小，默认 `4194304`
- `--warmup-mb`：正式录制前先做一次丢弃录制，默认 `4`；如果不需要可设为 `0`
- `--record-timeout`：等待 FPGA 录制完成的超时时间
- `--chunk-timeout`：每个 UDP 数据块读回超时
- `--iterations`：在同一个设备连接内连续录制的次数，默认 `1`
- `--overwrite`：多次录制时反复覆盖同一个 `--output` 文件；不指定时会追加 `.0001`、`.0002` 等序号

生成的 `e200_record.cs16` 可以直接作为回放输入文件。

连续录制 10 次、每次 128 MiB，并始终覆盖同一个文件：

```bash
./e200_record_iq_test \
  --addr 192.168.1.10 \
  --output e200_record.cs16 \
  --length-mb 128 \
  --warmup-mb 0 \
  --iterations 10 \
  --overwrite \
  --record-timeout 20 \
  --chunk-timeout 8
```

每轮录制完成、尚未开始 DMA 读回时，程序会检查 `readback_bytes=0`；完整读回后再检查其等于本轮录制长度。如果发现上一轮遗留的读回计数，程序会立即报错退出。

## IQ 回放

`e200_replay_iq_test` 会把一个 `cs16` 文件上传到 FPGA replay buffer，然后启动 replay。

生成一个 128 MiB 的相干复数单音文件，但暂不访问 SDR：

```bash
./e200_replay_iq_test \
  --generate-tone \
  --generate-only \
  --input e200_tone_128m.cs16 \
  --length-mb 128 \
  --sample-rate 30720000 \
  --tone-hz 1000000 \
  --tone-amplitude 0.25
```

程序会把请求的单音频率校准到最接近的相干频点，使文件首尾相位连续，避免循环边界产生额外跳变。

生成一个 128 MiB、基带频率从 `-5 MHz` 线性扫描到 `+5 MHz` 的周期 LFM 文件：

```bash
./e200_replay_iq_test \
  --generate-lfm \
  --generate-only \
  --input e200_lfm_128m.cs16 \
  --length-mb 128 \
  --sample-rate 30720000 \
  --lfm-start-hz -5000000 \
  --lfm-stop-hz 5000000 \
  --amplitude 0.25
```

128 MiB 在 30.72 MSPS 下包含 `33,554,432` 个复数样点，每轮扫描约 `1.09227 s`。程序会微调 LFM 中心频率到相干频点，使循环边界相位连续；边界处的瞬时频率从终止频率跳回起始频率，形成周期性锯齿扫频。

直接生成、上传并持续循环回放 LFM：

```bash
./e200_replay_iq_test \
  --addr 192.168.1.10 \
  --generate-lfm \
  --input e200_lfm_128m.cs16 \
  --length-mb 128 \
  --sample-rate 30720000 \
  --lfm-start-hz -5000000 \
  --lfm-stop-hz 5000000 \
  --amplitude 0.25 \
  --tx-lo 2400000000 \
  --tx-atten 89
```

上述射频扫频范围约为 `2395 MHz → 2405 MHz`。首次连接频谱仪验证时建议从较大的 TX 衰减开始，确认链路后再逐步降低衰减。

回放前面录制的文件：

```bash
./e200_replay_iq_test \
  --addr 192.168.1.10 \
  --input e200_tone_128m.cs16 \
  --sample-rate 30720000 \
  --tx-lo 2400000000 \
  --tx-atten 10
```

上传完成后 FPGA 会持续循环播放 DDR 中的数据。程序保持运行，按 `Ctrl+C` 后会向 SDR 发送 replay stop，再退出。
复数正频率单音的射频输出位于 `TX LO + tone Hz`；以上示例约为 `2401 MHz`。

也可以省略 `--generate-only`，用一条命令完成生成、上传和循环回放：

```bash
./e200_replay_iq_test \
  --addr 192.168.1.10 \
  --generate-tone \
  --input e200_tone_128m.cs16 \
  --length-mb 128 \
  --sample-rate 30720000 \
  --tone-hz 1000000 \
  --tone-amplitude 0.25 \
  --tx-lo 2400000000 \
  --tx-atten 10
```

常用参数：

- `--input`：输入 `cs16` 文件
- `--chunk`：上传分块大小，默认 `4194304`，最大 `4194304`
- `--packet-gap-us`：上传 replay IQ UDP 包之间的延时，默认 `25`
- `--chunk-timeout`：每个上传分块的超时时间，默认 `10`
- `--sample-rate`、`--tx-lo`、`--tx-atten`：回放时使用的射频参数

输入文件要求：

- 文件不能为空
- 文件大小必须 4 字节对齐
- 文件大小不能超过 `256 MiB`

如果需要重新回放另一个文件，建议先停止当前 replay，再上传并启动新的 replay。当前 host driver 的 `start_iq_replay()` 已经会在启动前发送 replay stop，用来避免第二次启动时旧状态影响 DMA。

当前 replay 会接管 DAC0 输出；不处于 replay 阶段时，DAC0 由 UOE TX 链路驱动。

## 常见检查

确认板端服务可达：

```bash
ping 192.168.1.10
```

查看示例程序参数：

```bash
./e200_iq_example --help
./e200_dual_rx_capture --help
./e200_record_iq_test --help
./e200_replay_iq_test --help
```

检查录制文件大小：

```bash
ls -lh e200_record.cs16
```

快速看 `cs16` 文件前几个样点：

```bash
od -t d2 -N 64 e200_record.cs16
```

如果 RX 没有持续出包，优先确认：

- 板端 `e200_v25_server` 是否正在运行
- 主机和板端 IP 是否在同一网段
- `--sample-rate`、`--rx-lo`、`--tx-lo` 是否和当前测试条件匹配
- RX 统计里的 `host drops`、`wire seq errors`、`wire ts errors`
- FPGA ILA 里的 `rx_mode_strobe`、`stream_start`、`current_mode`、`request_active`、`rx_tvalid`
