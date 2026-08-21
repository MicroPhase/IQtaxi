/*
 * Python bindings for GNU Radio 3.10+.
 */

#include <pybind11/pybind11.h>

#include <gnuradio/sync_block.h>
#include <gnuradio/iqtaxi/rx_source.h>
#include <gnuradio/iqtaxi/tx_sink.h>

namespace py = pybind11;

PYBIND11_MODULE(iqtaxi_python, m)
{
    py::module_::import("gnuradio.gr");

    m.doc() = "IQTAXI GNU Radio blocks";

    py::class_<
        gr::iqtaxi::rx_source,
        gr::sync_block,
        gr::iqtaxi::rx_source::sptr>(m, "rx_source")
        .def(py::init(&gr::iqtaxi::rx_source::make),
             py::arg("device") = "E100",
             py::arg("addr") = "192.168.1.10",
             py::arg("sample_rate") = 1000000.0,
             py::arg("center_freq") = 100000000.0,
             py::arg("gain") = 10.0,
             py::arg("samples_per_work") = 4096,
             py::arg("channels") = 1,
             py::arg("gain_ch1") = 10.0)
        .def("set_sample_rate", &gr::iqtaxi::rx_source::set_sample_rate)
        .def("set_center_freq", &gr::iqtaxi::rx_source::set_center_freq)
        .def("set_gain", &gr::iqtaxi::rx_source::set_gain)
        .def("set_gain_ch1", &gr::iqtaxi::rx_source::set_gain_ch1);

    py::class_<
        gr::iqtaxi::tx_sink,
        gr::sync_block,
        gr::iqtaxi::tx_sink::sptr>(m, "tx_sink")
        .def(py::init(&gr::iqtaxi::tx_sink::make),
             py::arg("device") = "E100",
             py::arg("addr") = "192.168.1.10",
             py::arg("sample_rate") = 1000000.0,
             py::arg("center_freq") = 100000000.0,
             py::arg("attenuation") = 0.0,
             py::arg("samples_per_packet") = 1024,
             py::arg("timed") = false,
             py::arg("start_delay_ms") = 0.0,
             py::arg("channels") = 1,
             py::arg("attenuation_ch1") = 0.0)
        .def("set_sample_rate", &gr::iqtaxi::tx_sink::set_sample_rate)
        .def("set_center_freq", &gr::iqtaxi::tx_sink::set_center_freq)
        .def("set_attenuation", &gr::iqtaxi::tx_sink::set_attenuation)
        .def("set_attenuation_ch1", &gr::iqtaxi::tx_sink::set_attenuation_ch1);
}
