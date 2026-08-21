/* -*- c++ -*- */
#ifndef INCLUDED_GR_IQTAXI_TX_SINK_H
#define INCLUDED_GR_IQTAXI_TX_SINK_H

#include <gnuradio/iqtaxi/api.h>
#include <gnuradio/iqtaxi/gr_compat.h>
#include <gnuradio/sync_block.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#if GR_VERSION_API < 10
#include <boost/shared_ptr.hpp>
#endif

namespace gr {
namespace iqtaxi {

class IQTAXI_API tx_sink : virtual public gr::sync_block
{
public:
#if GR_VERSION_API >= 10
    typedef std::shared_ptr<tx_sink> sptr;
#else
    typedef boost::shared_ptr<tx_sink> sptr;
#endif

    static sptr make(const std::string& device = "E100",
                     const std::string& addr = "192.168.1.10",
                     double sample_rate = 1000000.0,
                     double center_freq = 100000000.0,
                     double attenuation = 0.0,
                     std::size_t samples_per_packet = 1024,
                     bool timed = false,
                     double start_delay_ms = 0.0,
                     std::size_t channels = 1,
                     double attenuation_ch1 = 0.0);

    virtual void set_sample_rate(double sample_rate) = 0;
    virtual void set_center_freq(double center_freq) = 0;
    virtual void set_attenuation(double attenuation) = 0;
    virtual void set_attenuation_ch1(double attenuation) = 0;
};

} // namespace iqtaxi
} // namespace gr

#endif /* INCLUDED_GR_IQTAXI_TX_SINK_H */
