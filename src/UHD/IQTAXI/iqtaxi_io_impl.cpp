#include "iqtaxi_impl.hpp"
#include "iqtaxi_rx_streamer.hpp"
#include "iqtaxi_tx_streamer.hpp"

using namespace uhd;
using namespace uhd::usrp;


/***********************************************************************
 * Receive streamer
 **********************************************************************/
uhd::rx_streamer::sptr iqtaxi_impl::get_rx_stream(const uhd::stream_args_t& args_)
{
    std::lock_guard<std::mutex> lock(_transport_setup_mutex);

    UHD_LOGGER_INFO("IQTAXI RX streamer") << (boost::format("get rx streamer."));
    stream_args_t args = args_;

    // setup defaults for unspecified values
    if (args.otw_format.empty())
        args.otw_format = "sc16";
    args.channels = args.channels.empty() ? std::vector<size_t>(1, 0) : args.channels;
    if (_is_m300 && (args.channels.size() != 1u || args.channels[0] != 0u)) {
        throw uhd::value_error("M300 UHD RX currently supports channel 0 only");
    }

    std::shared_ptr<iqtaxi_rx_streamer> my_streamer;
    const size_t radio_index =
        _tree->access<std::vector<size_t>>("/mboards/0/rx_chan_dsp_mapping")
            .get()
            .at(args.channels[0]);
    const uint32_t sid =  e100_RX_DATA0_SID;

    radio_perifs_t& perif = _radio_perifs[0];
    // calculate packet size
    static const size_t hdr_size = 16;
    const size_t bpp = _is_m300 ? (16384u - hdr_size) :
        (_rx_stream_bus->get_xport()->get_recv_frame_size() - hdr_size);
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t wire_spp = bpp / bpi;
    const size_t stream_spp = _is_m300 ? wire_spp * 16u : wire_spp * 12u;
    size_t spp = unsigned(args.args.cast<double>("spp", wire_spp));
    spp = std::min(wire_spp, spp);
    std::cout << "spp:" << spp << " stream_spp:" << stream_spp << std::endl;

    // make the new streamer given the samples per packet
    if (not my_streamer)
        my_streamer = std::make_shared<iqtaxi_rx_streamer>(
            iqtaxi_device->get_rx_stream(), stream_spp, false);
    my_streamer->resize(args.channels.size());

    UHD_LOGGER_INFO("IQTAXI RX streamer") << (boost::format("get rx streamer."));

    // set the converter
    uhd::convert::id_type id;
    id.input_format  = args.otw_format + "_item32_le";
    id.num_inputs    = 1;
    id.output_format = args.cpu_format;
    id.num_outputs   = 1;
    my_streamer->set_converter(id);
    my_streamer->get_output_format(id.output_format);

    my_streamer->set_nsamps_per_packet(spp);
    my_streamer->set_rx_enable_chan(args.channels.size());
    my_streamer->set_tick_rate(_tick_rate);
    
    
    // _radio_perifs[0].rx_streamer = my_streamer; // store weak pointer
    perif.rx_streamer = my_streamer; // store weak pointer
    _rx_streamer = my_streamer;

    // sets all tick and samp rates on this streamer
    this->update_tick_rate(this->get_tick_rate());
    _tree
        ->access<double>(
            str(boost::format("/mboards/0/rx_dsps/%u/rate/value") % radio_index))
        .update();

    return my_streamer;
}

uhd::tx_streamer::sptr iqtaxi_impl::get_tx_stream(const uhd::stream_args_t& args_){
    std::lock_guard<std::mutex> lock(_transport_setup_mutex);

    stream_args_t args = args_;

    // setup defaults for unspecified values
    if (args.otw_format.empty())
        args.otw_format = "sc16";
    args.channels = args.channels.empty() ? std::vector<size_t>(1, 0) : args.channels;
    if (_is_m300 && (args.channels.size() != 1u || args.channels[0] != 0u)) {
        throw uhd::value_error("M300 UHD TX currently supports channel 0 only");
    }

    UHD_LOGGER_INFO("IQTAXI TX streamer") << (boost::format("get tx streamer."));

    std::shared_ptr<iqtaxi_tx_streamer> my_streamer;
    const size_t chan = args.channels[0];

    const size_t radio_index =
        _tree->access<std::vector<size_t>>("/mboards/0/tx_chan_dsp_mapping")
            .get()
            .at(args.channels[0]);
    radio_perifs_t& perif = _radio_perifs[radio_index];
     // calculate packet size
    static const size_t hdr_size = 16;
    const auto native_tx = iqtaxi_device->get_tx_stream();
    const size_t bpp = _is_m300 ? (16384u - hdr_size) :
        (_tx_stream_bus->get_xport()->get_send_frame_size() - hdr_size);
    const size_t wire_spp = bpp / convert::get_bytes_per_item(args.otw_format);
    const size_t spp = _is_m300 ? native_tx->get_max_num_samps() : wire_spp;


    // make the new streamer given the samples per packet
    if (not my_streamer) {
        if (_is_m300) {
            if (auto m300_tx = std::dynamic_pointer_cast<m300_tx_streamer>(native_tx)) {
                m300_tx->configure(wire_spp, false, 0x01u);
            }
        } else {
            // The E-series FPGA does not default to the host IQ stream.  UHD
            // callers (including srsRAN) only create a streamer and therefore
            // never perform the board-specific setup done by the native
            // examples.  Select host IQ, describe the UDP payload size and
            // keep timestamp scheduling enabled before the first packet.
            native_tx->set_tx_source(1u);
            _local_bus->poke32(
                CUSTOM_SET_TX_SAMPLES_PER_PACKET, static_cast<uint32_t>(wire_spp));
            _local_bus->poke32(
                CUSTOM_SET_TX_IGNORE_TIMESTAMPS, _ignore_tx_timestamps ? 1u : 0u);
        }
        my_streamer = std::make_shared<iqtaxi_tx_streamer>(
            native_tx, _local_bus, spp, false, iqtaxi_device, _is_m300);
    }
    my_streamer->resize(args.channels.size());

    UHD_LOGGER_INFO("IQTAXI TX streamer") << (boost::format("get tx streamer."));

    // set the converter
    uhd::convert::id_type id;
    id.input_format  = args.cpu_format;
    id.num_inputs    = 1;
    id.output_format = args.otw_format + "_item32_le";
    id.num_outputs   = 1;
    my_streamer->set_converter(id);
    my_streamer->get_input_format(id.input_format);
    my_streamer->set_tick_rate(_tick_rate);
    
    perif.tx_streamer = my_streamer; // store weak pointer
    _tx_streamer = my_streamer;
 
    this->update_tick_rate(this->get_tick_rate());
    _tree->access<double>(
                    str(boost::format("/mboards/0/tx_dsps/%u/rate/value") % radio_index))
            .update();

    return my_streamer;
}   

bool iqtaxi_impl::recv_async_msg(uhd::async_metadata_t& metadata, double timeout){
    if (auto streamer = _tx_streamer.lock()) {
        return streamer->recv_async_msg(metadata, timeout);
    }
    return false;
}
