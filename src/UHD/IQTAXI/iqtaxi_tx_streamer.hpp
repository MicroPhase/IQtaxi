#ifndef INCLUDE_IQTAXI_TX_STREAMER_HPP
#define INCLUDE_IQTAXI_TX_STREAMER_HPP

#include "iqtaxi_impl.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>

using namespace uhd;
using namespace uhd::transport;


class iqtaxi_tx_streamer : public uhd::tx_streamer {
public:
    iqtaxi_tx_streamer(sdr::api::tx_streamer::sptr tx_stream,
        local_ctrl::sptr local_bus,
        const size_t max_num_samps,
        bool big_endian,
        sdr::api::Device::sptr device = sdr::api::Device::sptr(),
        bool manage_stream = false,
        bool timed_send_per_burst_default = false):
    _tx_stream(tx_stream),
    _local_bus(local_bus),
    _device(device),
    _manage_stream(manage_stream),
    _timed_send_per_burst_default(timed_send_per_burst_default),
    _max_num_samps(max_num_samps),
    _bige(big_endian)
    {
        _max_num_samps = max_num_samps;
        _header_offset_words32 = 0;
        _queue_error_for_next_call = false;
        this->resize(1);
        if (_manage_stream) {
            _tx_stream->set_tx_source(1u);
            _tx_stream->set_stream_tx_start();
        }
    }

    ~iqtaxi_tx_streamer(){
        if (_manage_stream && _tx_stream) {
            try {
                _tx_stream->set_stream_tx_stop();
            } catch (...) {
            }
        }
    }

    size_t get_num_channels(void) const override
    {
        return _channel;
    }

    size_t get_max_num_samps(void) const override
    {
        return _max_num_samps;
    }

    //! Resize the number of transport channels
    void resize(const size_t size)
    {
        if (this->size() == size)
            return;
        _props.resize(size);
        static const uint64_t zero = 0;
        _zero_buffs.resize(size, &zero);
        _channel = size;
    }

    //! Get the channel width of this handler
    size_t size(void) const
    {
        return _props.size();
    }

    //! Set the rate of ticks per second
    void set_tick_rate(const double rate)
    {
        _tick_rate = rate;
        _samp_rate = rate;
    }

    //! Set the rate of samples per second
    void set_samp_rate(const double rate)
    {
        _samp_rate = rate;
    }

    void get_output_format(std::string id_output_format){
        output_format = id_output_format;
    }

    void get_input_format(std::string id_input_format){
        input_format = id_input_format;
    }

    void set_converter(const uhd::convert::id_type& id)
    {
        _num_outputs = id.num_outputs;
        _converter   = uhd::convert::get_converter(id)();
        this->set_scale_factor(1 / 32767.); // update after setting converter
        _bytes_per_otw_item = uhd::convert::get_bytes_per_item(id.input_format);
        _bytes_per_cpu_item = uhd::convert::get_bytes_per_item(id.output_format);
    }

    //! Set the scale factor used in float conversion
    void set_scale_factor(const double scale_factor)
    {
        _converter->set_scalar(scale_factor);
    }

    UHD_INLINE size_t send(const uhd::tx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t& metadata,
        const double timeout) override
    {
        (void)timeout;
        uhd::tx_metadata_t effective_metadata = metadata;

        // UHD permits a zero-length SOB call.  Cache it and attach it to the
        // next call that actually contains samples, matching UHD's native
        // super_send_packet_handler behaviour.
        if (nsamps_per_buff == 0 && metadata.start_of_burst) {
            trace_send("cache_sob", nsamps_per_buff, metadata, 0, 0);
            _cached_metadata = metadata;
            _metadata_cached = true;
            return 0;
        }
        if (_metadata_cached && nsamps_per_buff != 0) {
            if (!metadata.has_time_spec) {
                effective_metadata.has_time_spec = _cached_metadata.has_time_spec;
                effective_metadata.time_spec = _cached_metadata.time_spec;
            }
            effective_metadata.start_of_burst = _cached_metadata.start_of_burst;
            effective_metadata.end_of_burst =
                metadata.end_of_burst || _cached_metadata.end_of_burst;
            _metadata_cached = false;
        }

        // Some real-time applications (notably the legacy OAI LTE UE) submit
        // one complete, timestamped TTI per UHD send(), but only assert SOB on
        // the first call and never assert EOB while FDD is active.  E100 cannot
        // treat those calls as one indefinitely continuous FPGA burst: an
        // Ethernet/deep-FIFO gap would pause the samples and all later per-TTI
        // timestamps would be ignored.  Opt in to making every timestamped
        // non-empty send a self-contained burst, so the FPGA re-anchors each
        // TTI to its requested sample tick.
        if (timed_send_per_burst() && nsamps_per_buff != 0 &&
            effective_metadata.has_time_spec) {
            effective_metadata.start_of_burst = true;
            effective_metadata.end_of_burst = true;
        }

        const bool zero_length_eob =
            nsamps_per_buff == 0 && effective_metadata.end_of_burst;
        const size_t transport_nsamps = zero_length_eob ? 1 : nsamps_per_buff;
        const uhd::tx_streamer::buffs_type& transport_buffs =
            zero_length_eob ? _zero_buffs : buffs;

        size_t sent = 0;
        uint64_t send_timestamp = timestamp;

        if (effective_metadata.has_time_spec) {
            send_timestamp = effective_metadata.time_spec.to_ticks(_tick_rate);
            has_time = true;
        } else if (!has_time && _local_bus) {
            has_time = true;
            send_timestamp = _local_bus->peek64(CUSTOM_RB_GET_VITA_TIME_ADDR);
        } else if (!has_time && _device) {
            has_time = true;
            send_timestamp = _device->getTimeTicks();
        }

        uint64_t now_ticks = 0;
        const uint64_t trace_call = _trace_calls.load();
        if (trace_enabled() &&
            (trace_call < kMaxTraceCalls ||
             (trace_call % kTracePeriodCalls) == 0)) {
            if (_local_bus) {
                now_ticks = _local_bus->peek64(CUSTOM_RB_GET_VITA_TIME_ADDR);
            } else if (_device) {
                now_ticks = _device->getTimeTicks();
            }
        }
        trace_send("send", nsamps_per_buff, effective_metadata, send_timestamp, now_ticks);

        sdr::api::tx_packet_metadata packet_metadata;
        packet_metadata.flags_valid = true;
        packet_metadata.has_time = effective_metadata.has_time_spec;
        packet_metadata.start_of_burst = effective_metadata.start_of_burst;
        packet_metadata.end_of_burst = effective_metadata.end_of_burst;

        if (input_format == "sc16" || input_format == "cs16") {
            sent = _tx_stream->send_with_metadata(
                sdr::api::ref_vector<const void*>(&transport_buffs[0], transport_buffs.size()),
                transport_nsamps,
                send_timestamp,
                MICRORF_FORMAT_INT16,
                packet_metadata);
        } else if (input_format == "fc32" || input_format == "cf32") {
            sent = _tx_stream->send_with_metadata(
                sdr::api::ref_vector<const void*>(&transport_buffs[0], transport_buffs.size()),
                transport_nsamps,
                send_timestamp,
                MICRORF_FORMAT_FLOAT32,
                packet_metadata);
        } else {
            std::cout << "template cannot support this format" << std::endl;
            return 0;
        }

        // The transport advances send_timestamp by reference.  Adding sent a
        // second time placed every following untimed call too far in the
        // future.  A padded zero-length EOB is sent as one zero sample but, as
        // required by the UHD API, reports zero samples to the caller.
        timestamp = send_timestamp;
        return zero_length_eob ? 0 : sent;
    }

    bool recv_async_msg(uhd::async_metadata_t& metadata, double timeout) override
    {
        sdr::api::tx_async_event event;
        if (!_tx_stream->recv_async_event(event, timeout)) {
            return false;
        }

        metadata.channel = 0;
        metadata.has_time_spec = event.has_time;
        if (event.has_time) {
            metadata.time_spec = uhd::time_spec_t::from_ticks(event.timestamp, _tick_rate);
        }
        metadata.event_code = static_cast<uhd::async_metadata_t::event_code_t>(event.code);
        if (async_trace_enabled()) {
            std::cerr << "[IQTAXI_TX_TRACE] event=async code=0x" << std::hex
                      << static_cast<unsigned int>(event.code) << std::dec
                      << " has_time=" << (event.has_time ? 1 : 0)
                      << " tick=" << event.timestamp << std::endl;
        }
        metadata.user_payload[0] = 0;
        metadata.user_payload[1] = 0;
        metadata.user_payload[2] = 0;
        metadata.user_payload[3] = 0;
        return true;
    }

    void post_output_action(const std::shared_ptr<uhd::rfnoc::action_info>& action, const size_t port){

    }


private:
    static constexpr uint64_t kMaxTraceCalls = 40;
    static constexpr uint64_t kTracePeriodCalls = 1000;

    static bool trace_enabled()
    {
        static const bool enabled = std::getenv("IQTAXI_TX_TRACE") != nullptr;
        return enabled;
    }

    static bool async_trace_enabled()
    {
        static const bool enabled = trace_enabled() ||
                                    std::getenv("IQTAXI_TX_ASYNC_TRACE") != nullptr;
        return enabled;
    }

    bool timed_send_per_burst() const
    {
        const char* value = std::getenv("IQTAXI_TIMED_SEND_PER_BURST");
        if (value == nullptr) {
            return _timed_send_per_burst_default;
        }
        return std::string(value) != "0" && std::string(value) != "false";
    }

    void trace_send(const char* event,
                    size_t nsamps,
                    const uhd::tx_metadata_t& metadata,
                    uint64_t requested_tick,
                    uint64_t now_tick)
    {
        if (!trace_enabled()) {
            return;
        }
        const uint64_t call = _trace_calls.fetch_add(1);
        // Keep startup detail, then sample roughly once per second for OAI's
        // one-send-per-TTI path.  The old startup-only trace could not show a
        // late timestamp or shrinking lead that appeared tens of seconds
        // after attach.
        if (call >= kMaxTraceCalls && (call % kTracePeriodCalls) != 0) {
            return;
        }
        const int64_t lead = static_cast<int64_t>(requested_tick - now_tick);
        std::cerr << "[IQTAXI_TX_TRACE] call=" << call << " event=" << event
                  << " nsamps=" << nsamps
                  << " has_time=" << (metadata.has_time_spec ? 1 : 0)
                  << " sob=" << (metadata.start_of_burst ? 1 : 0)
                  << " eob=" << (metadata.end_of_burst ? 1 : 0)
                  << " requested=" << requested_tick
                  << " now=" << now_tick
                  << " lead=" << lead << std::endl;
    }

size_t _header_offset_words32;
    double _tick_rate, _samp_rate;
    bool _queue_error_for_next_call;
    size_t _alignment_failure_threshold;
    rx_metadata_t _queue_metadata;
    struct xport_chan_props_type
    {
        xport_chan_props_type(void)
            : packet_count(0)
        {
        }
        size_t packet_count;
    };
    std::vector<xport_chan_props_type> _props;
    size_t _num_outputs;
    size_t _bytes_per_otw_item; // used in conversion
    size_t _bytes_per_cpu_item; // used in conversion
    uhd::convert::converter::sptr _converter; // used in conversion
    std::string input_format,output_format;

    //! Shared variables for the worker threads
    size_t _convert_nsamps;
    const uhd::tx_streamer::buffs_type* _convert_buffs;
    size_t _convert_buffer_offset_bytes;
    size_t _convert_bytes_to_copy;

    std::atomic<bool> _rx_stopped{true};

    size_t _max_num_samps;

    const void * _copy_buff;
    uint32_t _max_sample_per_packet;
    size_t _request_num_samples;
    size_t _channel;
    sdr::api::tx_streamer::sptr _tx_stream;
    // double _tick_rate;
    const bool _bige;
    sdr_header_t _hdr;
    std::vector<const void*> _zero_buffs;
    uint64_t timestamp = 0; 
    local_ctrl::sptr _local_bus;
    sdr::api::Device::sptr _device;
    bool _manage_stream = false;
    bool _timed_send_per_burst_default = false;
    bool has_time = false;
    bool _metadata_cached = false;
    uhd::tx_metadata_t _cached_metadata;
    std::atomic<uint64_t> _trace_calls{0};
};

#endif
