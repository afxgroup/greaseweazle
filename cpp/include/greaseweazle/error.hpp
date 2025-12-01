/*
 * greaseweazle/error.hpp
 *
 * Error management and reporting.
 *
 * C++ port using libusb 0.1
 * Original Python code written & released by Keir Fraser <keir.xen@gmail.com>
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#ifndef GREASEWEAZLE_ERROR_HPP
#define GREASEWEAZLE_ERROR_HPP

#include <stdexcept>
#include <string>

namespace greaseweazle {

/**
 * @brief Fatal exception class for unrecoverable errors
 */
class Fatal : public std::runtime_error {
public:
    explicit Fatal(const std::string& message) 
        : std::runtime_error(message) {}
    
    explicit Fatal(const char* message) 
        : std::runtime_error(message) {}
};

/**
 * @brief Check a predicate and throw Fatal exception if false
 * @param pred The predicate to check
 * @param desc The error description if predicate is false
 */
inline void check(bool pred, const std::string& desc) {
    if (!pred) {
        throw Fatal(desc);
    }
}

inline void check(bool pred, const char* desc) {
    if (!pred) {
        throw Fatal(desc);
    }
}

} // namespace greaseweazle

#endif // GREASEWEAZLE_ERROR_HPP
