#include "ExtIO_Types.h"

#include "include/sdr/api/Device.hpp"
#include "src/driver/transport/local_regs.hpp"

#include <atomic>
#include <array>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <queue>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <process.h>
#  include <windows.h>
#endif

using sdr::api::Device;
using sdr::api::rx_streamer;

namespace {

constexpr size_t kMaxUdpPacketSamples = (1472u - 16u) / 4u;
// HDSDR needs large blocks. 512 @ 15.36 Msps => ~30k callbacks/s and eventually
// freezes/crashes. Keep a multiple of 512 (ExtIO rule). HackRF uses ~131072.
constexpr size_t kCallbackIqPairs = 2048u;
constexpr uint64_t kDefaultLoHz = 100000000ull;
constexpr uint32_t kDefaultRxGain = 40u;
constexpr size_t kNumCallbackBuffers = 4u;

// E100: 15.36 MHz family and 46.08 MHz family.
constexpr double kE100SampleRates[] = {
    122880000.0,
    61440000.0,
    30720000.0,
    15360000.0,
    7680000.0,
    3840000.0,
    1920000.0,
    46080000.0,
    23040000.0,
    11520000.0,
    5760000.0,
};

// E200 / AD9361: keep rates at or below 61.44 Msps.
constexpr double kE200SampleRates[] = {
    1920000.0,
    2000000.0,
    3840000.0,
    4000000.0,
    5000000.0,
    5760000.0,
    7680000.0,
    8000000.0,
    10000000.0,
    11520000.0,
    15360000.0,
    16000000.0,
    20000000.0,
    23040000.0,
    30720000.0,
    32000000.0,
    40000000.0,
    46080000.0,
    61440000.0,
};

// E206 firmware sample-rate profiles.
constexpr double kE206SampleRates[] = {
    122880000.0,
    61440000.0,
    30720000.0,
    15360000.0,
    7680000.0,
    3840000.0,
    1920000.0,
    46080000.0,
    23040000.0,
    11520000.0,
    5760000.0,
    80000000.0,
    40000000.0,
    20000000.0,
    10000000.0,
    5000000.0,
    64000000.0,
    32000000.0,
    16000000.0,
    8000000.0,
    4000000.0,
    2000000.0,
};

struct DeviceCaps {
    const double* sample_rates;
    int sample_rate_count;
    double default_sample_rate;
    uint32_t gain_min;
    uint32_t gain_max;
};

constexpr DeviceCaps kE100Caps = {
    kE100SampleRates,
    static_cast<int>(sizeof(kE100SampleRates) / sizeof(kE100SampleRates[0])),
    15360000.0,
    0u,
    41u,
};
constexpr DeviceCaps kE200Caps = {
    kE200SampleRates,
    static_cast<int>(sizeof(kE200SampleRates) / sizeof(kE200SampleRates[0])),
    30720000.0,
    0u,
    75u,
};
constexpr DeviceCaps kE206Caps = {
    kE206SampleRates,
    static_cast<int>(sizeof(kE206SampleRates) / sizeof(kE206SampleRates[0])),
    15360000.0,
    0u,
    42u,
};

const DeviceCaps& caps_for_device(const std::string& name)
{
    if (name.find("E200") != std::string::npos ||
        name.find("e200") != std::string::npos) {
        return kE200Caps;
    }
    if (name.find("E100") != std::string::npos ||
        name.find("e100") != std::string::npos) {
        return kE100Caps;
    }
    return kE206Caps;
}

int find_sample_rate_idx(const DeviceCaps& caps, double rate)
{
    for (int i = 0; i < caps.sample_rate_count; ++i) {
        if (caps.sample_rates[i] == rate) {
            return i;
        }
    }
    return 0;
}

uint32_t clamp_gain(const DeviceCaps& caps, uint32_t gain)
{
    if (gain < caps.gain_min) {
        return caps.gain_min;
    }
    if (gain > caps.gain_max) {
        return caps.gain_max;
    }
    return gain;
}

int gain_count(const DeviceCaps& caps)
{
    return static_cast<int>(caps.gain_max - caps.gain_min) + 1;
}

#ifdef _WIN32
// HDSDR does FFT work inside the ExtIO callback; commit a large stack.
constexpr unsigned kRxThreadStackBytes = 16u * 1024u * 1024u;
#  define EXTIO_EXPORT extern "C" __declspec(dllexport)
#else
#  define EXTIO_EXPORT extern "C" __attribute__((visibility("default")))
#endif

struct ExtioState {
    std::mutex mutex;
    pfnExtIOCallback callback = nullptr;

    std::string device_name = "E206";
    std::string addr = "192.168.1.10";
    const DeviceCaps* caps = &kE206Caps;
    std::atomic<uint32_t> sample_rate_hz{15360000u};
    std::atomic<uint64_t> lo_hz{kDefaultLoHz};
    std::atomic<uint32_t> rx_gain{kDefaultRxGain};
    std::atomic<int> sample_rate_idx{3};
    std::atomic<int> atten_idx{40};

    std::atomic<bool> pending_lo{false};
    std::atomic<bool> pending_gain{false};

    bool initialized = false;
    bool opened = false;
    bool running = false;

    Device::sptr device;
    rx_streamer::sptr rx_stream;

#ifdef _WIN32
    HANDLE rx_thread = nullptr;
    DWORD rx_tid = 0;
    HANDLE cb_thread = nullptr;
    DWORD cb_tid = 0;
#else
    std::thread rx_thread;
    std::thread cb_thread;
#endif
    std::atomic<bool> stop_rx{false};
    std::atomic<bool> stop_cb{false};

    // Decouple RX (network) thread from HDSDR callback thread.
    // RX fills a block and enqueues an index; cb thread calls deliver_iq().
    std::mutex cb_mutex;
    std::condition_variable cb_cv;
    std::queue<size_t> cb_free_indices;
    std::queue<size_t> cb_filled_indices;
    std::array<std::vector<int16_t>, kNumCallbackBuffers> cb_buffers;

    std::string host_name;
};

ExtioState& state()
{
    static ExtioState s;
    return s;
}

// Prefer process-wide atomic over MinGW emutls thread_local: a stuck TLS depth
// would silently drop every IQ callback while HDSDR UI keeps scrolling.
std::atomic<int> g_host_callback_depth{0};

struct HostCallbackGuard {
    HostCallbackGuard() { g_host_callback_depth.fetch_add(1); }
    ~HostCallbackGuard() { g_host_callback_depth.fetch_sub(1); }
};

bool in_host_callback()
{
    return g_host_callback_depth.load() > 0;
}

#ifdef _WIN32
bool on_rx_thread(const ExtioState& st)
{
    return st.rx_tid != 0 && GetCurrentThreadId() == st.rx_tid;
}

bool on_cb_thread(const ExtioState& st)
{
    return st.cb_tid != 0 && GetCurrentThreadId() == st.cb_tid;
}
#endif

void log_msg(const char* msg)
{
#ifdef _WIN32
    char path[MAX_PATH];
    DWORD n = GetTempPathA(MAX_PATH, path);
    if (n == 0 || n + 20 >= MAX_PATH) {
        return;
    }
    std::memcpy(path + n, "ExtIO_MicroPhase.log", 21);
    FILE* f = std::fopen(path, "a");
    if (!f) {
        return;
    }
    SYSTEMTIME st{};
    GetLocalTime(&st);
    std::fprintf(f,
                 "%04u-%02u-%02u %02u:%02u:%02u.%03u %s\n",
                 st.wYear,
                 st.wMonth,
                 st.wDay,
                 st.wHour,
                 st.wMinute,
                 st.wSecond,
                 st.wMilliseconds,
                 msg);
    std::fclose(f);
#else
    (void)msg;
#endif
}

void notify_status(int status)
{
    pfnExtIOCallback cb = nullptr;
    {
        std::lock_guard<std::mutex> lock(state().mutex);
        cb = state().callback;
    }
    if (!cb) {
        return;
    }
    HostCallbackGuard guard;
    cb(-1, status, 0.0f, nullptr);
}

void deliver_iq(pfnExtIOCallback cb, int16_t* iq)
{
    // Never skip IQ delivery — a stuck reentrancy guard previously caused
    // "waterfall still moves but spectrum frozen" after a while.
    HostCallbackGuard guard;
    cb(static_cast<int>(kCallbackIqPairs), 0, 0.0f, iq);
}

std::string env_or(const char* key, const char* fallback)
{
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    return value;
}

void apply_defaults_from_env(ExtioState& st)
{
    st.device_name = env_or("IQTAXI_EXTIO_DEVICE", "E206");
    st.addr = env_or("IQTAXI_EXTIO_ADDR", "192.168.1.10");
    st.caps = &caps_for_device(st.device_name);
    st.sample_rate_idx.store(find_sample_rate_idx(*st.caps, st.caps->default_sample_rate));
    st.sample_rate_hz.store(static_cast<uint32_t>(st.caps->default_sample_rate));
    const uint32_t gain = clamp_gain(*st.caps, kDefaultRxGain);
    st.rx_gain.store(gain);
    st.atten_idx.store(static_cast<int>(gain - st.caps->gain_min));
}

bool configure_rf_locked(ExtioState& st)
{
    if (!st.device) {
        return false;
    }
    st.device->set_channel_enable(1u);
    st.device->set_dma_mode(0u);
    st.device->setSampleRate(static_cast<double>(st.sample_rate_hz.load()));
    st.device->set_rx_freq(st.lo_hz.load(), 1);
    st.device->set_rx_gain(st.rx_gain.load(), 1);
    return true;
}

void apply_pending_rf_updates()
{
    auto& st = state();
    Device::sptr device;
    uint64_t lo = 0;
    uint32_t gain = 0;
    bool do_lo = false;
    bool do_gain = false;

    {
        std::lock_guard<std::mutex> lock(st.mutex);
        device = st.device;
        if (!device) {
            return;
        }
        if (st.pending_lo.exchange(false)) {
            lo = st.lo_hz.load();
            do_lo = true;
        }
        if (st.pending_gain.exchange(false)) {
            gain = st.rx_gain.load();
            do_gain = true;
        }
    }

    try {
        if (do_lo) {
            device->set_rx_freq(lo, 1);
        }
        if (do_gain) {
            device->set_rx_gain(gain, 1);
        }
    } catch (...) {
    }
}

void stop_stream_resources_locked(ExtioState& st)
{
    st.running = false;
    if (st.rx_stream) {
        try {
            uint64_t stop_timestamp = 0;
            st.rx_stream->set_recv_param(STREAM_MODE, kCallbackIqPairs, stop_timestamp, 0, 1);
            st.rx_stream->set_rx_mode_exit();
        } catch (...) {
        }
        st.rx_stream.reset();
    }
}

void rx_worker();
void callback_worker();

void request_stop_rx(ExtioState& st)
{
    st.stop_rx.store(true);
}

void request_stop_rx_locked(ExtioState& st)
{
    request_stop_rx(st);
}

#ifdef _WIN32
unsigned __stdcall rx_worker_trampoline(void*)
{
    rx_worker();
    return 0;
}

unsigned __stdcall callback_worker_trampoline(void*)
{
    callback_worker();
    return 0;
}

void reap_rx_thread(ExtioState& st, DWORD timeout_ms)
{
    if (st.rx_thread == nullptr) {
        return;
    }
    WaitForSingleObject(st.rx_thread, timeout_ms);
    CloseHandle(st.rx_thread);
    st.rx_thread = nullptr;
    st.rx_tid = 0;
}

void reap_cb_thread(ExtioState& st, DWORD timeout_ms)
{
    if (st.cb_thread == nullptr) {
        return;
    }
    WaitForSingleObject(st.cb_thread, timeout_ms);
    CloseHandle(st.cb_thread);
    st.cb_thread = nullptr;
    st.cb_tid = 0;
}

bool start_rx_thread(ExtioState& st)
{
    reap_rx_thread(st, 5000);
    st.stop_rx.store(false);

    unsigned tid = 0;
    uintptr_t handle = _beginthreadex(
        nullptr,
        kRxThreadStackBytes,
        rx_worker_trampoline,
        nullptr,
        0,
        &tid);
    if (handle == 0) {
        st.rx_tid = 0;
        return false;
    }
    st.rx_thread = reinterpret_cast<HANDLE>(handle);
    st.rx_tid = tid;
    return true;
}

bool start_cb_thread(ExtioState& st)
{
    reap_cb_thread(st, 5000);
    st.stop_cb.store(false);

    {
        std::lock_guard<std::mutex> lock(st.cb_mutex);
        while (!st.cb_free_indices.empty()) {
            st.cb_free_indices.pop();
        }
        while (!st.cb_filled_indices.empty()) {
            st.cb_filled_indices.pop();
        }
        for (size_t i = 0; i < kNumCallbackBuffers; ++i) {
            auto& b = st.cb_buffers[i];
            b.resize(kCallbackIqPairs * 2u);
            st.cb_free_indices.push(i);
        }
    }

    unsigned tid = 0;
    uintptr_t handle = _beginthreadex(
        nullptr,
        kRxThreadStackBytes,
        callback_worker_trampoline,
        nullptr,
        0,
        &tid);
    if (handle == 0) {
        st.cb_tid = 0;
        return false;
    }
    st.cb_thread = reinterpret_cast<HANDLE>(handle);
    st.cb_tid = tid;
    return true;
}

void join_rx_thread(ExtioState& st)
{
    request_stop_rx(st);
    if (on_rx_thread(st)) {
        // StopHW from inside HDSDR callback: cannot join self.
        return;
    }
    reap_rx_thread(st, INFINITE);
}

void join_cb_thread(ExtioState& st)
{
    st.stop_cb.store(true);
    st.cb_cv.notify_all();
    if (on_cb_thread(st)) {
        // StopHW from inside HDSDR callback: cannot join self.
        return;
    }
    reap_cb_thread(st, INFINITE);
}
#else
bool start_rx_thread(ExtioState& st)
{
    if (st.rx_thread.joinable()) {
        return false;
    }
    st.stop_rx.store(false);
    st.rx_thread = std::thread(rx_worker);
    return true;
}

void join_rx_thread(ExtioState& st)
{
    request_stop_rx(st);
    if (st.rx_thread.joinable()) {
        st.rx_thread.join();
    }
}

bool start_cb_thread(ExtioState& st)
{
    if (st.cb_thread.joinable()) {
        return false;
    }
    st.stop_cb.store(false);
    {
        std::lock_guard<std::mutex> lock(st.cb_mutex);
        while (!st.cb_free_indices.empty()) {
            st.cb_free_indices.pop();
        }
        while (!st.cb_filled_indices.empty()) {
            st.cb_filled_indices.pop();
        }
        for (size_t i = 0; i < kNumCallbackBuffers; ++i) {
            auto& b = st.cb_buffers[i];
            b.resize(kCallbackIqPairs * 2u);
            st.cb_free_indices.push(i);
        }
    }
    st.cb_thread = std::thread(callback_worker);
    return true;
}

void join_cb_thread(ExtioState& st)
{
    st.stop_cb.store(true);
    st.cb_cv.notify_all();
    if (st.cb_thread.joinable()) {
        st.cb_thread.join();
    }
}
#endif

void recover_rx_stream(rx_streamer::sptr rx)
{
    if (!rx) {
        return;
    }
    uint64_t ts = 0;
    try {
        // Full stop joins the UDP producer thread and drains FIFO (releases frames).
        rx->set_recv_param(STREAM_MODE, kCallbackIqPairs, ts, 0, 1);
    } catch (...) {
    }

    try {
        rx->set_rx_mode(STREAM_MODE);
        rx->set_max_sample_nums_per_packet(kMaxUdpPacketSamples);
        rx->set_recv_param(STREAM_MODE, kCallbackIqPairs, ts, 1, 0);
        log_msg("rx_worker recovered STREAM_MODE");
    } catch (...) {
        log_msg("rx_worker recover failed");
    }
}

void callback_worker()
{
    auto& st = state();
    log_msg("callback_worker start");

    while (true) {
        size_t buf_idx = 0u;
        {
            std::unique_lock<std::mutex> lock(st.cb_mutex);
            st.cb_cv.wait(lock, [&] {
                return st.stop_cb.load() || !st.cb_filled_indices.empty();
            });
            if (st.stop_cb.load() && st.cb_filled_indices.empty()) {
                break;
            }
            if (st.cb_filled_indices.empty()) {
                continue;
            }
            buf_idx = st.cb_filled_indices.front();
            st.cb_filled_indices.pop();
        }

        pfnExtIOCallback cb = nullptr;
        {
            std::lock_guard<std::mutex> lock(st.mutex);
            cb = st.callback;
        }
        if (cb) {
            deliver_iq(cb, st.cb_buffers[buf_idx].data());
        }

        {
            std::lock_guard<std::mutex> lock(st.cb_mutex);
            st.cb_free_indices.push(buf_idx);
        }
    }

    log_msg("callback_worker exit");
}

void rx_worker()
{
    auto& st = state();
    log_msg("rx_worker start");

    std::vector<int16_t> iq(kCallbackIqPairs * 2u);
    std::vector<int16_t> recv_buf(kCallbackIqPairs * 2u);
    std::vector<void*> buffs{recv_buf.data()};
    size_t filled = 0u;
    uint64_t blocks = 0u;
    uint32_t zero_reads = 0u;

    {
        std::lock_guard<std::mutex> lock(st.mutex);
        if (!st.rx_stream) {
            log_msg("rx_worker abort: no stream");
            return;
        }
        try {
            uint64_t timestamp = 0;
            st.rx_stream->set_rx_mode(STREAM_MODE);
            st.rx_stream->set_max_sample_nums_per_packet(kMaxUdpPacketSamples);
            st.rx_stream->set_recv_param(STREAM_MODE, kCallbackIqPairs, timestamp, 1, 0);
        } catch (...) {
            log_msg("rx_worker stream setup exception");
        }
    }

    while (!st.stop_rx.load()) {
        apply_pending_rf_updates();

        rx_streamer::sptr rx;
        pfnExtIOCallback cb = nullptr;
        {
            std::lock_guard<std::mutex> lock(st.mutex);
            rx = st.rx_stream;
            cb = st.callback;
        }
        if (!rx || !cb) {
            break;
        }

        uint64_t timestamp = 0;
        size_t received = 0;
        try {
            received = rx->recv(buffs, kCallbackIqPairs - filled, timestamp, MICRORF_FORMAT_INT16);
        } catch (...) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (received == 0) {
            ++zero_reads;
            // ~1s of idle (recv waits up to 20ms): fully restart data path.
            if (zero_reads >= 50u) {
                zero_reads = 0u;
                filled = 0u;
                recover_rx_stream(rx);
            }
            continue;
        }
        zero_reads = 0u;

        if (received > (kCallbackIqPairs - filled)) {
            received = kCallbackIqPairs - filled;
        }

        std::memcpy(iq.data() + filled * 2u,
                    recv_buf.data(),
                    received * 2u * sizeof(int16_t));
        filled += received;

        if (filled < kCallbackIqPairs) {
            continue;
        }

        // Enqueue one IQ block for the callback thread.
        // If all callback buffers are busy, drop this block to keep RX real-time.
        size_t buf_idx = 0u;
        bool have_buf = false;
        {
            std::lock_guard<std::mutex> lock(st.cb_mutex);
            if (!st.cb_free_indices.empty()) {
                buf_idx = st.cb_free_indices.front();
                st.cb_free_indices.pop();
                have_buf = true;
            }
        }
        if (have_buf) {
            std::memcpy(st.cb_buffers[buf_idx].data(),
                        iq.data(),
                        kCallbackIqPairs * 2u * sizeof(int16_t));
            {
                std::lock_guard<std::mutex> lock(st.cb_mutex);
                st.cb_filled_indices.push(buf_idx);
            }
            st.cb_cv.notify_one();
        }

        filled = 0u;
        ++blocks;
        if ((blocks % 2048u) == 0u) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "rx_worker blocks=%llu", static_cast<unsigned long long>(blocks));
            log_msg(buf);
        }
    }

    log_msg("rx_worker exit");
}

} // namespace

EXTIO_EXPORT bool EXTIO_CALL InitHW(char *name, char *model, int &hwtype)
{
    auto& st = state();
    std::lock_guard<std::mutex> lock(st.mutex);
    apply_defaults_from_env(st);

    if (name) {
        std::snprintf(name, 16, "MicroPhase");
    }
    if (model) {
        std::snprintf(model, 16, "%s", st.device_name.c_str());
    }
    hwtype = exthwUSBdata16;
    st.initialized = true;
    log_msg("InitHW");
    return true;
}

EXTIO_EXPORT bool EXTIO_CALL OpenHW(void)
{
    auto& st = state();
    std::lock_guard<std::mutex> lock(st.mutex);
    if (!st.initialized) {
        return false;
    }
    if (st.opened) {
        return true;
    }

    try {
        st.device = Device::makeDevice(st.device_name, st.addr);
        if (!st.device) {
            log_msg("OpenHW makeDevice failed");
            return false;
        }
        if (!configure_rf_locked(st)) {
            st.device.reset();
            log_msg("OpenHW configure_rf failed");
            return false;
        }
        st.opened = true;
        {
            const std::string opened =
                std::string("OpenHW ok ") +
                sdr::api::e100_display_name(st.device->get_profile());
            log_msg(opened.c_str());
        }
        return true;
    } catch (...) {
        st.device.reset();
        log_msg("OpenHW exception");
        return false;
    }
}

EXTIO_EXPORT void EXTIO_CALL CloseHW(void)
{
    auto& st = state();
    {
        std::lock_guard<std::mutex> lock(st.mutex);
        if (st.running) {
            request_stop_rx_locked(st);
        }
    }
    join_rx_thread(st);
    join_cb_thread(st);
    {
        std::lock_guard<std::mutex> lock(st.mutex);
        stop_stream_resources_locked(st);
        st.device.reset();
        st.opened = false;
    }
    log_msg("CloseHW");
    notify_status(extHw_Disconnected);
}

EXTIO_EXPORT int EXTIO_CALL StartHW(long extLOfreq)
{
    auto& st = state();
    {
        std::lock_guard<std::mutex> lock(st.mutex);
        if (!st.opened || !st.device || st.running) {
            return 0;
        }

        try {
            if (extLOfreq > 0) {
                st.lo_hz.store(static_cast<uint64_t>(extLOfreq));
            }
            configure_rf_locked(st);
            st.rx_stream = st.device->get_rx_stream();
            if (!st.rx_stream) {
                return 0;
            }

            st.pending_lo.store(false);
            st.pending_gain.store(false);
            st.running = true;
        } catch (...) {
            st.rx_stream.reset();
            st.running = false;
            return 0;
        }
    }

    if (!start_rx_thread(st)) {
        std::lock_guard<std::mutex> lock(st.mutex);
        st.rx_stream.reset();
        st.running = false;
        log_msg("StartHW thread failed");
        return 0;
    }

    // Start callback thread after RX thread is ready.
    if (!start_cb_thread(st)) {
        std::lock_guard<std::mutex> lock(st.mutex);
        join_rx_thread(st);
        st.rx_stream.reset();
        st.running = false;
        log_msg("StartHW cb thread failed");
        return 0;
    }
    log_msg("StartHW ok");
    return static_cast<int>(kCallbackIqPairs);
}

EXTIO_EXPORT void EXTIO_CALL StopHW(void)
{
    auto& st = state();
    {
        std::lock_guard<std::mutex> lock(st.mutex);
        if (!st.running) {
            return;
        }
        request_stop_rx_locked(st);
    }
    join_rx_thread(st);
    join_cb_thread(st);
    {
        std::lock_guard<std::mutex> lock(st.mutex);
        stop_stream_resources_locked(st);
    }
    log_msg("StopHW");
}

EXTIO_EXPORT void EXTIO_CALL SetCallback(pfnExtIOCallback funcptr)
{
    auto& st = state();
    std::lock_guard<std::mutex> lock(st.mutex);
    st.callback = funcptr;
}

EXTIO_EXPORT int EXTIO_CALL SetHWLO(long extLOfreq)
{
    auto& st = state();
    if (extLOfreq <= 0) {
        return -1;
    }

    const uint64_t new_lo = static_cast<uint64_t>(extLOfreq);
    const uint64_t old_lo = st.lo_hz.exchange(new_lo);
    if (new_lo == old_lo) {
        return 0;
    }

    if (in_host_callback()) {
        st.pending_lo.store(true);
        return 0;
    }

    try {
        std::lock_guard<std::mutex> lock(st.mutex);
        if (st.device) {
            st.device->set_rx_freq(new_lo, 1);
        }
        st.pending_lo.store(false);
        return 0;
    } catch (...) {
        return 1;
    }
}

EXTIO_EXPORT long EXTIO_CALL GetHWLO(void)
{
    const uint64_t lo = state().lo_hz.load();
    if (lo > static_cast<uint64_t>(0x7fffffff)) {
        return 0x7fffffff;
    }
    return static_cast<long>(lo);
}

EXTIO_EXPORT long EXTIO_CALL GetHWSR(void)
{
    return static_cast<long>(state().sample_rate_hz.load());
}

EXTIO_EXPORT int EXTIO_CALL GetStatus(void)
{
    auto& st = state();
    std::lock_guard<std::mutex> lock(st.mutex);
    if (st.running) {
        return extHw_RUNNING;
    }
    if (st.opened) {
        return extHw_READY;
    }
    return extHw_Disconnected;
}

EXTIO_EXPORT void EXTIO_CALL ShowGUI(void)
{
}

EXTIO_EXPORT void EXTIO_CALL HideGUI(void)
{
}

EXTIO_EXPORT int EXTIO_CALL ExtIoGetSrates(int idx, double *samplerate)
{
    const DeviceCaps& caps = *state().caps;
    if (idx < 0 || idx >= caps.sample_rate_count) {
        return -1;
    }
    if (samplerate) {
        *samplerate = caps.sample_rates[idx];
    }
    return 0;
}

EXTIO_EXPORT int EXTIO_CALL ExtIoGetActualSrateIdx(void)
{
    return state().sample_rate_idx.load();
}

EXTIO_EXPORT int EXTIO_CALL ExtIoSetSrate(int idx)
{
    auto& st = state();
    const DeviceCaps& caps = *st.caps;
    if (idx < 0 || idx >= caps.sample_rate_count) {
        return -1;
    }

    if (in_host_callback()) {
        return -1;
    }

    const uint32_t new_rate = static_cast<uint32_t>(caps.sample_rates[idx]);
    bool restart = false;
    {
        std::lock_guard<std::mutex> lock(st.mutex);
        if (st.sample_rate_idx.load() == idx && st.sample_rate_hz.load() == new_rate) {
            return 0;
        }
        restart = st.running;
        if (restart) {
            request_stop_rx_locked(st);
        }
    }
    join_rx_thread(st);

    {
        std::lock_guard<std::mutex> lock(st.mutex);
        stop_stream_resources_locked(st);
        st.sample_rate_idx.store(idx);
        st.sample_rate_hz.store(new_rate);
        try {
            if (st.device) {
                st.device->setSampleRate(static_cast<double>(st.sample_rate_hz.load()));
            }
        } catch (...) {
            return -1;
        }

        if (restart && st.opened && st.device) {
            try {
                st.rx_stream = st.device->get_rx_stream();
                st.running = true;
            } catch (...) {
                st.running = false;
                return -1;
            }
        } else {
            restart = false;
        }
    }

    if (restart && !start_rx_thread(st)) {
        std::lock_guard<std::mutex> lock(st.mutex);
        st.rx_stream.reset();
        st.running = false;
        return -1;
    }
    return 0;
}

EXTIO_EXPORT int EXTIO_CALL GetAttenuators(int idx, float *attenuation)
{
    const DeviceCaps& caps = *state().caps;
    if (idx < 0 || idx >= gain_count(caps)) {
        return -1;
    }
    if (attenuation) {
        *attenuation = static_cast<float>(caps.gain_min + static_cast<uint32_t>(idx));
    }
    return 0;
}

EXTIO_EXPORT int EXTIO_CALL GetActualAttIdx(void)
{
    return state().atten_idx.load();
}

EXTIO_EXPORT int EXTIO_CALL SetAttenuator(int idx)
{
    auto& st = state();
    const DeviceCaps& caps = *st.caps;
    if (idx < 0 || idx >= gain_count(caps)) {
        return -1;
    }

    st.atten_idx.store(idx);
    const uint32_t new_gain = caps.gain_min + static_cast<uint32_t>(idx);
    const uint32_t old_gain = st.rx_gain.exchange(new_gain);
    if (new_gain == old_gain) {
        return 0;
    }

    if (in_host_callback()) {
        st.pending_gain.store(true);
        return 0;
    }

    try {
        std::lock_guard<std::mutex> lock(st.mutex);
        if (st.device) {
            st.device->set_rx_gain(st.rx_gain.load(), 1);
        }
        st.pending_gain.store(false);
        return 0;
    } catch (...) {
        return -1;
    }
}

EXTIO_EXPORT void EXTIO_CALL VersionInfo(const char *progname, int ver_major, int ver_minor)
{
    auto& st = state();
    std::lock_guard<std::mutex> lock(st.mutex);
    if (progname) {
        st.host_name = progname;
    }
    (void)ver_major;
    (void)ver_minor;
}
