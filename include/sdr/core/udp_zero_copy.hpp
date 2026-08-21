//
// Created by jcc on 25-4-8.
//

#ifndef SOAPY_UDP_ZERO_COPY_HPP
#define SOAPY_UDP_ZERO_COPY_HPP

#ifdef _WIN32
    #ifndef NOMINMAX
    #define NOMINMAX
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
#endif
#include <functional>
#include <vector>
#include <string>
#include "memory"

#pragma once
#include <string>
#include "zero_copy.hpp"

namespace sdr {namespace core{
        class API_EXPORT udp_zero_copy : public zero_copy_if{
        public:
            typedef std::shared_ptr<udp_zero_copy> sptr;

            static sptr make(const std::string& addr,const std::string &port,
                    const zero_copy_xport_params& default_buff_args);

            virtual uint16_t get_local_port(void) const = 0;

            virtual std::string get_local_addr(void) const = 0;
        };
}}



#endif //SOAPY_UDP_ZERO_COPY_HPP
