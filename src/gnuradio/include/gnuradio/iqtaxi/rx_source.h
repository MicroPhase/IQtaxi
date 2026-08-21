/* -*- c++ -*- */
#ifndef INCLUDED_GR_IQTAXI_RX_SOURCE_H
#define INCLUDED_GR_IQTAXI_RX_SOURCE_H

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

class IQTAXI_API rx_source : virtual public gr::sync_block
{
public:
#if GR_VERSION_API >= 10
    typedef std::shared_ptr<rx_source> sptr;
#else
    typedef boost::shared_ptr<rx_source> sptr;
#endif

    static sptr make(const std::string& device = "E100",
                     const std::string& addr = "192.168.1.10",
                     double sample_rate = 1000000.0,
                     double center_freq = 100000000.0,
                     double gain = 10.0,
                     std::size_t samples_per_work = 4096,
                     std::size_t channels = 1,
                     double gain_ch1 = 10.0);

    virtual void set_sample_rate(double sample_rate) = 0;
    virtual void set_center_freq(double center_freq) = 0;
    virtual void set_gain(double gain) = 0;
    virtual void set_gain_ch1(double gain) = 0;
};

} // namespace iqtaxi
} // namespace gr

#endif /* INCLUDED_GR_IQTAXI_RX_SOURCE_H */
