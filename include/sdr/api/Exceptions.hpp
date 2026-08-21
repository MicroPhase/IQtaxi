//
// Created by jcc on 25-4-8.
//

#ifndef SOAPY_EXCEPTIONS_HPP
#define SOAPY_EXCEPTIONS_HPP

#pragma once
#include <stdexcept>

namespace sdr::api {

    class SdrException : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    class DeviceNotFoundError : public SdrException {
    public:
        explicit DeviceNotFoundError(const std::string& uri)
                : SdrException("Device not found: " + uri) {}
    };

    class StreamConfigError : public SdrException {
    public:
        explicit StreamConfigError(const std::string& msg)
                : SdrException("Stream config error: " + msg) {}
    };

} // namespace sdr::api

#endif //SOAPY_EXCEPTIONS_HPP
