#ifndef SOAPY_UDP_COMMON_HPP
#define SOAPY_UDP_COMMON_HPP

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#ifdef _WIN32
    #ifndef NOMINMAX
    #define NOMINMAX
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>

    using sdr_socket_t = SOCKET;
    using sdr_socket_len_t = int;
    using sdr_socket_ret_t = int;
    constexpr sdr_socket_t SDR_INVALID_SOCKET = INVALID_SOCKET;

    typedef ULONG nfds_t;
    #define CLOSE_SOCKET(s) closesocket(s)

    inline int poll(struct pollfd* fds, nfds_t nfds, int timeout)
    {
        return WSAPoll(fds, nfds, timeout);
    }
#else
    #include <arpa/inet.h>
    #include <poll.h>
    #include <sys/socket.h>
    #include <unistd.h>

    using sdr_socket_t = int;
    using sdr_socket_len_t = socklen_t;
    using sdr_socket_ret_t = ssize_t;
    constexpr sdr_socket_t SDR_INVALID_SOCKET = -1;

    #define CLOSE_SOCKET(s) close(s)
#endif

namespace sdr { namespace core {

        constexpr size_t MAX_ETHERNET_MTU = 9600;
        constexpr size_t UDP_DEFAULT_NUM_FRAMES = 1;
        constexpr size_t UDP_DEFAULT_FRAME_SIZE = 1472;
        constexpr size_t UDP_DEFAULT_BUFF_SIZE = 2500000;

        struct UdpSocketInfo {
            sdr_socket_t sock_fd;
            std::string addr;
            std::string port;

            UdpSocketInfo(sdr_socket_t fd, const std::string& address, const std::string& port)
                    : sock_fd(fd), addr(address), port(port) {}
        };

        typedef std::shared_ptr<UdpSocketInfo> socket_sptr;

        inline bool wait_for_recv_ready(sdr_socket_t sock_fd, int32_t timeout_ms)
        {
            struct pollfd pfd_read;
            pfd_read.fd = sock_fd;
            pfd_read.events = POLLIN;

            return poll(&pfd_read, 1, timeout_ms) > 0;
        }

        inline socket_sptr open_udp_socket(const std::string& addr, const std::string& port)
        {
            sdr_socket_t sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock_fd == SDR_INVALID_SOCKET) {
                throw std::runtime_error("Error creating socket");
            }

            int buff_size = 64 * 1024 * 1024;
            if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&buff_size), sizeof(buff_size)) < 0) {
                perror("setsockopt SO_RCVBUF");
            }

            struct sockaddr_in receiver_endpoint;
            receiver_endpoint.sin_family = AF_INET;
            receiver_endpoint.sin_port = htons(std::stoi(port));
            receiver_endpoint.sin_addr.s_addr = inet_addr(addr.c_str());

            if (connect(sock_fd, reinterpret_cast<struct sockaddr*>(&receiver_endpoint), sizeof(receiver_endpoint)) < 0) {
                CLOSE_SOCKET(sock_fd);
                throw std::runtime_error("Error connecting to server");
            }

            return std::make_shared<UdpSocketInfo>(sock_fd, addr, port);
        }

        inline size_t recv_udp_packet(sdr_socket_t sock_fd, void* mem, size_t frame_size, int32_t timeout_ms)
        {
            if (wait_for_recv_ready(sock_fd, timeout_ms)) {
                const int recv_size = static_cast<int>(std::min<size_t>(frame_size, static_cast<size_t>((std::numeric_limits<int>::max)())));
                sdr_socket_ret_t len = recv(sock_fd, static_cast<char*>(mem), recv_size, 0);
                if (len == 0) {
                    throw std::runtime_error("Socket closed");
                }
                if (len < 0) {
                    throw std::runtime_error("Error receiving packet");
                }
                return static_cast<size_t>(len);
            }
            return 0;
        }

        inline void send_udp_packet(sdr_socket_t sock_fd, void* mem, size_t len)
        {
            const int send_size = static_cast<int>(std::min<size_t>(len, static_cast<size_t>((std::numeric_limits<int>::max)())));
            while (true) {
                sdr_socket_ret_t ret = send(sock_fd, static_cast<const char*>(mem), send_size, 0);
                if (ret == send_size) {
                    break;
                }
#ifdef _WIN32
                if (ret == SOCKET_ERROR && WSAGetLastError() == WSAENOBUFS) {
#else
                if (ret == -1 && errno == ENOBUFS) {
#endif
                    std::this_thread::sleep_for(std::chrono::microseconds(1));
                    continue;
                }
                if (ret == static_cast<sdr_socket_ret_t>(-1)) {
                    throw std::runtime_error("Error sending packet");
                }
            }
        }

        inline size_t get_udp_socket_buffer_size(sdr_socket_t sock_fd, int level, int option)
        {
            size_t option_value;
            sdr_socket_len_t opt_len = sizeof(option_value);
            if (getsockopt(sock_fd, level, option, reinterpret_cast<char*>(&option_value), &opt_len) < 0) {
                throw std::runtime_error("Error getting socket buffer size");
            }
            return option_value;
        }

        inline size_t resize_udp_socket_buffer(sdr_socket_t sock_fd, size_t num_bytes, int level, int option)
        {
            if (setsockopt(sock_fd, level, option, reinterpret_cast<const char*>(&num_bytes), sizeof(num_bytes)) < 0) {
                throw std::runtime_error("Error setting socket buffer size");
            }

            return get_udp_socket_buffer_size(sock_fd, level, option);
        }

        inline size_t resize_udp_socket_buffer_with_warning(
                std::function<size_t(size_t)> resize_fn,
                const size_t target_size,
                const std::string& name)
        {
            size_t actual_size = 0;
            if (target_size > 0) {
                actual_size = resize_fn(target_size);
            }
            return actual_size;
        }

    }} // namespace sdr::core

#endif // SOAPY_UDP_COMMON_HPP
