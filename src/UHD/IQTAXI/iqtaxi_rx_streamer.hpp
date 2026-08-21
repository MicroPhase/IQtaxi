#ifndef INCLUDE_IQTAXI_RX_STREAMER_HPP
#define INCLUDE_IQTAXI_RX_STREAMER_HPP

#include <uhd/rfnoc/actions.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

using namespace uhd;

class iqtaxi_rx_streamer : public uhd::rx_streamer {
    public:
    iqtaxi_rx_streamer(sdr::api::rx_streamer::sptr rx_stream,
        const size_t max_num_samps,
        bool big_endian):
        _rx_stream(rx_stream),
        _max_num_samps(max_num_samps),
        _stream_request_samps(max_num_samps),
        _packet_samps(max_num_samps),
        _active_stream_samps(max_num_samps),
        _trace_id(next_trace_id()),
        _bige(big_endian)
    {
        _max_num_samps = max_num_samps;
        _header_offset_words32 = 0;
        _queue_error_for_next_call=false;
        this->resize(1);
        trace("construct", "max_samps=" + std::to_string(max_num_samps));
    }

    ~iqtaxi_rx_streamer() override
    {
        trace("destruct", _rx_stopped.load() ? "already_stopped=1" : "already_stopped=0");
        stop_stream_noexcept();
    }

    size_t get_num_channels(void) const override
    {
        return _channel;
    }

    size_t get_max_num_samps(void) const override
    {
        return _max_num_samps;
    }

    void issue_stream_cmd(const stream_cmd_t& stream_cmd) override
    {
        trace("issue_cmd",
              "mode=" + std::to_string(static_cast<int>(stream_cmd.stream_mode)) +
                  " now=" + std::to_string(stream_cmd.stream_now ? 1 : 0) +
                  " num=" + std::to_string(stream_cmd.num_samps) +
                  " rate=" + std::to_string(static_cast<uint64_t>(_samp_rate)));
        uint32_t rx_mode = 0;
        if (stream_cmd.stream_mode == stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
                || stream_cmd.stream_mode == stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE)
        {
            _rx_stopped = false;
            if (stream_cmd.stream_now)
            {
                rx_mode = PACKET_MODE;
                _rx_stream->set_rx_mode(rx_mode);
                _rx_stream->set_rx_sample_nums_per_packet(stream_cmd.num_samps);
                _rx_stream->set_stream_rx_start();
            } else {
                rx_mode = SYNC_MODE;
                _rx_stream->set_rx_mode(rx_mode);
                uint64_t ticks =
                    (stream_cmd.stream_now) ? 0 : stream_cmd.time_spec.to_ticks(_tick_rate);
                _rx_stream->set_recv_param(rx_mode,stream_cmd.num_samps,ticks,1,0);
            }
        } else if (stream_cmd.stream_mode == stream_cmd_t::STREAM_MODE_START_CONTINUOUS) {
                rx_mode = STREAM_MODE;
                // Define a new hardware stream epoch even if a previous host
                // exited without sending STOP.  This prevents old packets and
                // an old sequence counter from straddling a new UHD session.
                uint64_t stop_timestamp = 0;
                _rx_stream->set_recv_param(
                    STREAM_MODE, _active_stream_samps, stop_timestamp, 0, 1);
                _rx_stream->set_rx_mode_exit();
                // Allow the board's final in-flight UDP datagrams to reach the
                // host before the native START path drains the transport.
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                _rx_stream->set_rx_mode(rx_mode);
                uint64_t timestampe = 0;
                _active_stream_samps = aligned_stream_samps(_stream_request_samps);
                _rx_stream->set_recv_param(STREAM_MODE, _active_stream_samps, timestampe, 1, 0);
                _rx_stopped = false;
                _first_packet_logged = false;
                _timeout_logs = 0;
                trace("start_done", "active_samps=" + std::to_string(_active_stream_samps));
        } else if (stream_cmd.stream_mode == stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS) {
                _rx_stopped = true;
                uint64_t timestampe = 0;
                _rx_stream->set_recv_param(STREAM_MODE, _active_stream_samps, timestampe, 0, 1);
                _rx_stream->set_rx_mode_exit();
                trace("stop_done", "active_samps=" + std::to_string(_active_stream_samps));
        }

    }

    void get_output_format(std::string id_output_format){
        output_format = id_output_format;
    }


    UHD_INLINE size_t recv(const uhd::rx_streamer::buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet)override
    {
        metadata = uhd::rx_metadata_t();

        if (buffs.size() == 0 || nsamps_per_buff == 0) {
            metadata.error_code = rx_metadata_t::ERROR_CODE_NONE;
            return 0;
        }

        if(_rx_stopped.load()){
            const double wait_time = std::max(0.0, std::min(timeout, kStoppedRecvSleepSec));
            if (wait_time > 0.0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
            }
            metadata.end_of_burst = true;
            metadata.error_code = rx_metadata_t::ERROR_CODE_TIMEOUT;
            return 0;
        }

        uint32_t sample_format = 0;
        if(output_format == "sc16" || output_format == "cs16") {
            sample_format = MICRORF_FORMAT_INT16;
        } else if(output_format == "fc32" || output_format == "cf32") {
            sample_format = MICRORF_FORMAT_FLOAT32;
        } else {
            metadata.error_code = rx_metadata_t::ERROR_CODE_BAD_PACKET;
            return 0;
        }

        _rx_stream->set_sample_format(sample_format);

        uint64_t time_stamp = 0;
        int flags = 0;
        const size_t samples_to_recv =
            one_packet ? std::min(nsamps_per_buff, _packet_samps) : nsamps_per_buff;
        const long timeout_us = timeout_to_us(timeout);
        const size_t total_recv_samples = _rx_stream->recv_from_fifo_ticks(
            &buffs[0], samples_to_recv, flags, time_stamp, timeout_us);

        if(total_recv_samples == 0 && samples_to_recv > 0){
            if (_timeout_logs++ < 3) {
                trace("recv_timeout",
                      "request=" + std::to_string(samples_to_recv) +
                          " timeout_us=" + std::to_string(timeout_us));
            }
            metadata.error_code = rx_metadata_t::ERROR_CODE_TIMEOUT;
            metadata.end_of_burst = false;
            return 0;
        }

        // Preserve the FPGA clock exactly.  Hiding a wire discontinuity by
        // synthesizing timestamps on the host separates RX metadata from
        // get_time_now() and timed TX, which breaks LTE TTI alignment.
        if ((flags & IQTAXI_RX_FIFO_FLAG_DISCONTINUITY_BEFORE) != 0) {
            metadata.error_code = rx_metadata_t::ERROR_CODE_OVERFLOW;
            metadata.out_of_sequence = true;
        } else {
            metadata.error_code = rx_metadata_t::ERROR_CODE_NONE;
        }
        metadata.time_spec = uhd::time_spec_t::from_ticks(time_stamp, _samp_rate);
        metadata.has_time_spec = true;
        if (!_first_packet_logged.exchange(true)) {
            trace("first_packet",
                  "samples=" + std::to_string(total_recv_samples) +
                      " tick=" + std::to_string(time_stamp) +
                      " flags=" + std::to_string(flags));
        }
        return total_recv_samples;
    }

    void set_rx_enable_chan(uint8_t chans) const
    {
        _rx_stream->set_rx_enable_chan(chans);
    }

    void set_max_sample_per_packet(uint32_t max_sample_per_packet){
        _rx_stream->set_rx_sample_nums_per_packet(max_sample_per_packet); 
    }


    void resize(const size_t size)
    {
        _channel = size;
        _props.resize(size);
        // re-initialize all buffers infos by re-creating the vector
        // _buffers_infos = std::vector<buffers_info_type>(4, buffers_info_type(size));
    }
    
    void set_converter(const uhd::convert::id_type& id)
    {
        _num_outputs = id.num_outputs;
        _converter   = uhd::convert::get_converter(id)();
        this->set_scale_factor(1 / 32767.); // update after setting converter
        _bytes_per_otw_item = uhd::convert::get_bytes_per_item(id.input_format);
        _bytes_per_cpu_item = uhd::convert::get_bytes_per_item(id.output_format);
    }

    void set_nsamps_per_user_buffer(const size_t nsamps)
    {
        _stream_request_samps = aligned_stream_samps(nsamps);
    }

    void set_nsamps_per_packet(const size_t nsamps)
    {
        _packet_samps = aligned_packet_samps(nsamps);
        _rx_stream->set_max_sample_nums_per_packet(_packet_samps);
        _stream_request_samps = aligned_stream_samps(_stream_request_samps);
        _active_stream_samps = aligned_stream_samps(_active_stream_samps);
    }

    //! Set the scale factor used in float conversion
    void set_scale_factor(const double scale_factor)
    {
        _converter->set_scalar(scale_factor);
    }

    //! Set the rate of ticks per second
    void set_tick_rate(const double rate)
    {
        const double old_rate = _samp_rate;
        _tick_rate = rate;
        _samp_rate = rate;
        _rx_stream->set_sampleRate(static_cast<size_t>(rate));
        trace("set_rate",
              "old=" + std::to_string(static_cast<uint64_t>(old_rate)) +
                  " new=" + std::to_string(static_cast<uint64_t>(rate)));
    }

    void post_input_action(const std::shared_ptr<uhd::rfnoc::action_info>& action, const size_t port){

    }
private:
    static constexpr double kStoppedRecvSleepSec = 0.1;
    static constexpr long kMaxRecvTimeoutUs = 30000000;

    static bool trace_enabled()
    {
        static const bool enabled = std::getenv("IQTAXI_RX_TRACE") != nullptr;
        return enabled;
    }

    static uint64_t next_trace_id()
    {
        static std::atomic<uint64_t> next{1};
        return next.fetch_add(1);
    }

    void trace(const char* event, const std::string& detail) const
    {
        if (trace_enabled()) {
            std::cerr << "[IQTAXI_RX_TRACE] id=" << _trace_id << " event=" << event
                      << " " << detail << std::endl;
        }
    }

    void stop_stream_noexcept() noexcept
    {
        if (_rx_stopped.exchange(true)) {
            return;
        }
        try {
            uint64_t timestamp = 0;
            _rx_stream->set_recv_param(
                STREAM_MODE, _active_stream_samps, timestamp, 0, 1);
            _rx_stream->set_rx_mode_exit();
        } catch (...) {
            // Destructors must not throw; the next START also forces a fresh
            // hardware epoch as recovery for an interrupted process.
        }
    }

    size_t aligned_stream_samps(const size_t nsamps) const
    {
        if (_packet_samps == 0) {
            return nsamps;
        }
        const size_t packets = std::max<size_t>(
            1, (nsamps + _packet_samps - 1) / _packet_samps);
        return packets * _packet_samps;
    }

    size_t aligned_packet_samps(const size_t nsamps) const
    {
        return std::max<size_t>(8, nsamps - (nsamps % 8));
    }

    long timeout_to_us(const double timeout) const
    {
        if (timeout < 0.0) {
            return kMaxRecvTimeoutUs;
        }
        if (timeout == 0.0) {
            return 1;
        }
        const double timeout_us = timeout * 1e6;
        if (timeout_us > static_cast<double>(kMaxRecvTimeoutUs)) {
            return kMaxRecvTimeoutUs;
        }
        return std::max<long>(1, static_cast<long>(timeout_us));
    }

    size_t _header_offset_words32;
    double _tick_rate{0.0}, _samp_rate{0.0};
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
    std::string output_format;

    //! Shared variables for the worker threads
    size_t _convert_nsamps;
    const rx_streamer::buffs_type* _convert_buffs;
    size_t _convert_buffer_offset_bytes;
    size_t _convert_bytes_to_copy;

    std::atomic<bool> _rx_stopped{true};
    std::atomic<bool> _first_packet_logged{false};
    std::atomic<unsigned int> _timeout_logs{0};

    size_t _max_num_samps;
    size_t _stream_request_samps;
    size_t _packet_samps;
    size_t _active_stream_samps;
    const uint64_t _trace_id;

    const void * _copy_buff;
    uint32_t _max_sample_per_packet;
    size_t _request_num_samples;
    size_t _channel;
    sdr::api::rx_streamer::sptr _rx_stream;
    // double _tick_rate;
    const bool _bige;
    sdr_header_t _hdr;
};

#endif
