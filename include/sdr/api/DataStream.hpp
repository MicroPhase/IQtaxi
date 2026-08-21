//
// Created by jcc on 25-4-8.
//

#ifndef SOAPY_DATASTREAM_HPP
#define SOAPY_DATASTREAM_HPP

#include "../config.hpp"
#include "memory"
#include "vector"


namespace sdr{namespace api{
    template <typename T>
    class API_EXPORT ref_vector
    {
        public:
        template <typename Ptr>
        ref_vector(Ptr* ptr) : _ptr(T(ptr)), _mem(_mem_t(&_ptr)), _size(1)
        {
            /* NOP */
        }
        template <typename Vector>
        ref_vector(const Vector& vec)
        : _ptr(T()), _mem(_mem_t(&vec.front())), _size(vec.size())
        {
            /* NOP */
        }

        /*!
         * Create a reference vector from a pointer and a length
         * Therefore: rv[n] == mem[n] and rv.size() == len
         * \param mem a pointer to an array of pointers
         * \param len the length of the array of pointers
         */
        ref_vector(const T* mem, size_t len) : _ptr(T()), _mem(_mem_t(mem)), _size(len)
        {
            /* NOP */
        }

        //! Index operator gets the value of rv[index]
        const T& operator[](size_t index) const
        {
            return _mem[index];
        }

        //! The number of elements in this container
        size_t size(void) const
        {
            return _size;
        }

        private:
        const T _ptr;
        typedef T* _mem_t;
        const _mem_t _mem;
        const size_t _size;
    };

    class API_EXPORT rx_streamer
    {
    public:
        typedef std::shared_ptr<rx_streamer> sptr;

        virtual ~rx_streamer(){}

        //! Get the number of channels associated with this streamer
        virtual size_t get_num_channels(void) const = 0;

        //! Get the max number of samples per buffer per packet
        virtual size_t get_request_num_samps(void) const = 0;

        //! Set the rx enabled channels
        virtual void set_rx_enable_chan(uint8_t chans) const = 0;

        virtual void set_rx_mode(uint8_t mode) = 0;
        virtual void set_rx_mode_exit() = 0;
        virtual void set_stream_rx_start() = 0;
        virtual void set_stream_rx_stop() = 0;

        virtual void set_max_sample_nums_per_packet(uint32_t sample_nums) = 0;
        virtual void set_rx_sample_nums_per_packet(uint32_t sample_nums) = 0;

        //! Typedef for a pointer to a single, or a collection of recv buffers
        typedef ref_vector<void*> buffs_type;

        virtual size_t set_recv_param(uint8_t rx_mode,
                            const size_t nsamps_per_buff,
                            uint64_t &time_stamp,
                            uint8_t stream_start,
                            uint8_t stream_stop
                            ) = 0;

        virtual size_t recv(const buffs_type& buffs,
                            const size_t nsamps_per_buff,
                            uint64_t &time_stamp,
                            uint32_t sample_format) = 0;

        virtual size_t recv_from_fifo(void * const *buffs,
                            const size_t numElems,
                            int &flags,
                            long long &timeNs,
                            const long timeoutUs) = 0;

        // Native sample-clock timestamp path.  Timed-radio consumers must not
        // round FPGA ticks through nanoseconds because the result is also used
        // to schedule future TX bursts in the same hardware time domain.
        virtual size_t recv_from_fifo_ticks(void * const *buffs,
                            const size_t numElems,
                            int &flags,
                            uint64_t &timeTicks,
                            const long timeoutUs) = 0;


        virtual size_t recv_fifo(const buffs_type& buffs,
                                const size_t nsamps_per_buff) = 0;
        
        virtual void rx_thread_func(uint32_t buff_size_samples) = 0;

        virtual void _start(void) = 0;
        virtual void _stop(void) = 0;;

        virtual void enable_xfft(const size_t fft_point) = 0;
        virtual void set_sample_format(uint32_t format) = 0;
        virtual void set_sampleRate(size_t sampleRate) = 0;

    };

   struct tx_packet_metadata {
       bool flags_valid = false;
       bool has_time = false;
       bool start_of_burst = false;
       bool end_of_burst = false;
   };

   enum class tx_async_event_code : uint32_t {
       burst_ack = 0x1,
       underflow = 0x2,
       sequence_error = 0x4,
       time_error = 0x8
   };

   struct tx_async_event {
       tx_async_event_code code = tx_async_event_code::burst_ack;
       bool has_time = false;
       uint64_t timestamp = 0;
   };

   class API_EXPORT tx_streamer {
   public:
       typedef std::shared_ptr<tx_streamer> sptr;

       virtual ~tx_streamer() {}

       virtual size_t get_num_channels(void) const = 0;
    
       virtual void set_tx_source(uint32_t){

       };

       virtual size_t get_max_num_samps(void) const = 0;

       typedef ref_vector<const void *> buffs_type;

       virtual size_t send(const buffs_type &buffs,
                           const size_t nsamps_per_buff,
                           uint64_t &time_stamp,
                            uint32_t sample_format) = 0;

       // Burst-aware extension used by UHD.  Native/legacy callers keep using
       // send(), while E-series transports can preserve UHD SOB/EOB/has-time
       // metadata in the existing 16-byte wire header.
       virtual size_t send_with_metadata(const buffs_type &buffs,
                           const size_t nsamps_per_buff,
                           uint64_t &time_stamp,
                           uint32_t sample_format,
                           const tx_packet_metadata &metadata)
       {
           (void)metadata;
           return send(buffs, nsamps_per_buff, time_stamp, sample_format);
       }

       virtual size_t send_nonblocking(const buffs_type &buffs,
                           const size_t nsamps_per_buff,
                           uint64_t &time_stamp,
                           uint32_t sample_format)
        {
            return send(buffs, nsamps_per_buff, time_stamp, sample_format);
        }

        virtual bool recv_async_event(tx_async_event&, double)
        {
            return false;
        }

        virtual void dds_ctrl(uint32_t phase_ctrl) = 0;

        virtual void set_stream_tx_start() = 0;
        virtual void set_stream_tx_stop() = 0;
   };

}}


#endif //SOAPY_DATASTREAM_HPP
