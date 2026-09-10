# iq_taxi

## 编译并安装 Linux 驱动

标准的 CMake/Make 安装流程会自动安装一套相互匹配的
`libsdr_core`、`libsdr_driver` 和 UHD `libIQTaxiUHD` 插件：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local
make -C build -j"$(nproc)"
sudo make -C build install
```

`sudo make install` 末尾还会检查三者是否都已安装并自动执行
`ldconfig`。不要只复制 UHD 插件，否则插件与运行库版本不一致时，
GNU Radio/gr-osmosdr 可能在创建 RX streamer 时崩溃。已有构建目录若曾
使用其他安装前缀，请先重新运行上面的 `cmake -S ...` 配置命令。

## E100 sample rates

E100 上位机驱动与当前 legacy-rate 下位机一致，支持以下 10 档采样率：

`1.92 / 3.84 / 5.76 / 7.68 / 11.52 / 15.36 / 23.04 / 30.72 / 61.44 / 122.88 MSPS`

`E100Impl::setSampleRate()` 会像 E206 一样将请求量化到最近的支持档位。公共定义在
`include/sdr/api/SampleRates.hpp`，E100/E206 核心驱动、E100 GUI、SoapySDR、
SDR++ 和 SDRangel 都从这里取得 MP2021 采样率能力表。

## E200 examples

E200 上位机示例程序使用方法见：

- [E200_EXAMPLES_USAGE.md](E200_EXAMPLES_USAGE.md)
