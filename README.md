# iq_taxi

## E100 sample rates

E100 上位机驱动与当前 legacy-rate 下位机一致，支持以下 10 档采样率：

`1.92 / 3.84 / 5.76 / 7.68 / 11.52 / 15.36 / 23.04 / 30.72 / 61.44 / 122.88 MSPS`

`E100Impl::setSampleRate()` 会像 E206 一样将请求量化到最近的支持档位。公共定义在
`include/sdr/api/SampleRates.hpp`，E100/E206 核心驱动、E100 GUI、SoapySDR、
SDR++ 和 SDRangel 都从这里取得 MP2021 采样率能力表。

## E200 examples

E200 上位机示例程序使用方法见：

- [E200_EXAMPLES_USAGE.md](E200_EXAMPLES_USAGE.md)
