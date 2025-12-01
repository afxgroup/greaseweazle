/*
 * greaseweazle/usb.hpp
 *
 * USB communication with Greaseweazle device using libusb 0.1
 *
 * C++ port using libusb 0.1
 * Original Python code written & released by Keir Fraser <keir.xen@gmail.com>
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#ifndef GREASEWEAZLE_USB_HPP
#define GREASEWEAZLE_USB_HPP

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <tuple>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <thread>

#define __USE_INLINE__
#include <usb.h>  // libusb 0.1

#include "error.hpp"
#include "flux.hpp"

namespace greaseweazle {

// Earliest supported firmware version
constexpr std::pair<int, int> EARLIEST_SUPPORTED_FIRMWARE = {0, 31};

/**
 * @brief Control-Path command set for baud rate signaling
 */
namespace ControlCmd {
    constexpr int ClearComms = 10000;
    constexpr int Normal = 9600;
}

/**
 * @brief Command set for Greaseweazle protocol
 */
namespace Cmd {
    constexpr uint8_t GetInfo = 0;
    constexpr uint8_t Update = 1;
    constexpr uint8_t Seek = 2;
    constexpr uint8_t Head = 3;
    constexpr uint8_t SetParams = 4;
    constexpr uint8_t GetParams = 5;
    constexpr uint8_t Motor = 6;
    constexpr uint8_t ReadFlux = 7;
    constexpr uint8_t WriteFlux = 8;
    constexpr uint8_t GetFluxStatus = 9;
    constexpr uint8_t GetIndexTimes = 10;
    constexpr uint8_t SwitchFwMode = 11;
    constexpr uint8_t Select = 12;
    constexpr uint8_t Deselect = 13;
    constexpr uint8_t SetBusType = 14;
    constexpr uint8_t SetPin = 15;
    constexpr uint8_t Reset = 16;
    constexpr uint8_t EraseFlux = 17;
    constexpr uint8_t SourceBytes = 18;
    constexpr uint8_t SinkBytes = 19;
    constexpr uint8_t GetPin = 20;
    constexpr uint8_t TestMode = 21;
    constexpr uint8_t NoClickStep = 22;

    inline const char* to_string(uint8_t cmd) {
        switch (cmd) {
            case GetInfo: return "GetInfo";
            case Update: return "Update";
            case Seek: return "Seek";
            case Head: return "Head";
            case SetParams: return "SetParams";
            case GetParams: return "GetParams";
            case Motor: return "Motor";
            case ReadFlux: return "ReadFlux";
            case WriteFlux: return "WriteFlux";
            case GetFluxStatus: return "GetFluxStatus";
            case GetIndexTimes: return "GetIndexTimes";
            case SwitchFwMode: return "SwitchFwMode";
            case Select: return "Select";
            case Deselect: return "Deselect";
            case SetBusType: return "SetBusType";
            case SetPin: return "SetPin";
            case Reset: return "Reset";
            case EraseFlux: return "EraseFlux";
            case SourceBytes: return "SourceBytes";
            case SinkBytes: return "SinkBytes";
            case GetPin: return "GetPin";
            case TestMode: return "TestMode";
            case NoClickStep: return "NoClickStep";
            default: return "Unknown";
        }
    }
}

/**
 * @brief Command responses/acknowledgements
 */
namespace Ack {
    constexpr uint8_t Okay = 0;
    constexpr uint8_t BadCommand = 1;
    constexpr uint8_t NoIndex = 2;
    constexpr uint8_t NoTrk0 = 3;
    constexpr uint8_t FluxOverflow = 4;
    constexpr uint8_t FluxUnderflow = 5;
    constexpr uint8_t Wrprot = 6;
    constexpr uint8_t NoUnit = 7;
    constexpr uint8_t NoBus = 8;
    constexpr uint8_t BadUnit = 9;
    constexpr uint8_t BadPin = 10;
    constexpr uint8_t BadCylinder = 11;
    constexpr uint8_t OutOfSRAM = 12;
    constexpr uint8_t OutOfFlash = 13;

    inline const char* to_string(uint8_t code) {
        switch (code) {
            case Okay: return "Okay";
            case BadCommand: return "Bad Command";
            case NoIndex: return "No Index";
            case NoTrk0: return "Track 0 not found";
            case FluxOverflow: return "Flux Overflow";
            case FluxUnderflow: return "Flux Underflow";
            case Wrprot: return "Disk is Write Protected";
            case NoUnit: return "No drive unit selected";
            case NoBus: return "No bus type (eg. Shugart, IBM/PC) specified";
            case BadUnit: return "Invalid unit number";
            case BadPin: return "Invalid pin";
            case BadCylinder: return "Invalid cylinder";
            case OutOfSRAM: return "Out of SRAM";
            case OutOfFlash: return "Out of Flash";
            default: return "Unknown Error";
        }
    }
}

/**
 * @brief Cmd.GetInfo indexes
 */
namespace GetInfo {
    constexpr uint8_t Firmware = 0;
    constexpr uint8_t BandwidthStats = 1;
    constexpr uint8_t CurrentDrive = 7;
}

/**
 * @brief Cmd.{Get,Set}Params indexes
 */
namespace Params {
    constexpr uint8_t Delays = 0;
}

/**
 * @brief Bus type enumeration
 */
enum class BusType : uint8_t {
    Invalid = 0,
    IBMPC = 1,
    Shugart = 2
};

/**
 * @brief Flux read stream opcodes
 */
namespace FluxOp {
    constexpr uint8_t Index = 1;
    constexpr uint8_t Space = 2;
    constexpr uint8_t Astable = 3;
}

/**
 * @brief Cmd.GetInfo DriveInfo result
 */
struct DriveInfo {
    static constexpr uint32_t FLAG_CYL_VALID = 1;
    static constexpr uint32_t FLAG_MOTOR_ON = 2;
    static constexpr uint32_t FLAG_IS_FLIPPY = 4;

    std::optional<int32_t> cyl;
    bool motor_on;
    bool is_flippy;

    explicit DriveInfo(const uint8_t* rsp) {
        uint32_t flags;
        int32_t cyl_val;
        std::memcpy(&flags, rsp, sizeof(flags));
        std::memcpy(&cyl_val, rsp + 4, sizeof(cyl_val));
        
        if (flags & FLAG_CYL_VALID) {
            cyl = cyl_val;
        }
        motor_on = (flags & FLAG_MOTOR_ON) != 0;
        is_flippy = (flags & FLAG_IS_FLIPPY) != 0;
    }

    std::string to_string() const {
        std::string s = "Cyl: ";
        s += cyl.has_value() ? std::to_string(*cyl) : "Unknown";
        if (motor_on) s += "; Motor-On";
        if (is_flippy) s += "; Is-Flippy";
        return s;
    }
};

/**
 * @brief Command error exception
 */
class CmdError : public std::exception {
public:
    std::vector<uint8_t> cmd;
    uint8_t code;

    CmdError(const std::vector<uint8_t>& cmd, uint8_t code)
        : cmd(cmd), code(code) {}

    std::string cmd_str() const {
        return cmd.empty() ? "UnknownCmd" : Cmd::to_string(cmd[0]);
    }

    std::string errcode_str() const {
        if (code == Ack::BadCylinder && !cmd.empty() && cmd[0] == Cmd::Seek) {
            std::string s = Ack::to_string(Ack::BadCylinder);
            if (cmd.size() >= 3) {
                int8_t cyl;
                if (cmd.size() == 3) {
                    cyl = static_cast<int8_t>(cmd[2]);
                } else {
                    int16_t cyl16;
                    std::memcpy(&cyl16, &cmd[2], sizeof(cyl16));
                    cyl = static_cast<int8_t>(cyl16);
                }
                s += " " + std::to_string(cyl);
            }
            return s;
        }
        return Ack::to_string(code);
    }

    const char* what() const noexcept override {
        static thread_local std::string msg;
        msg = cmd_str() + ": " + errcode_str();
        return msg.c_str();
    }
};

/**
 * @brief USB GreaseweazleUnit class for Greaseweazle communication using libusb 0.1
 */
class GreaseweazleUnit {
private:
    usb_dev_handle* dev_handle_;
    int endpoint_in_;
    int endpoint_out_;
    int interface_;

    // Helper to read exactly n bytes
    std::vector<uint8_t> read_bytes(int count, int timeout_ms = 5000) {
        std::vector<uint8_t> buffer(count);
        int total_read = 0;
        while (total_read < count) {
            int ret = usb_bulk_read(dev_handle_, endpoint_in_,
                                    reinterpret_cast<char*>(buffer.data() + total_read),
                                    count - total_read, timeout_ms);
            if (ret < 0) {
                throw Fatal(std::string("USB read error: ") + usb_strerror());
            }
            total_read += ret;
        }
        return buffer;
    }

    // Helper to write bytes
    void write_bytes(const std::vector<uint8_t>& data) {
        int total_written = 0;
        while (total_written < static_cast<int>(data.size())) {
            int ret = usb_bulk_write(dev_handle_, endpoint_out_,
                                    (char *)(data.data() + total_written),
                                     data.size() - total_written, 5000);
            if (ret < 0) {
                throw Fatal(std::string("USB write error: ") + usb_strerror());
            }
            total_written += ret;
        }
    }

    void write_bytes(const uint8_t* data, size_t len) {
        int total_written = 0;
        while (total_written < static_cast<int>(len)) {
            int ret = usb_bulk_write(dev_handle_, endpoint_out_,
                                     (char *)(data + total_written),
                                     len - total_written, 5000);
            if (ret < 0) {
                throw Fatal(std::string("USB write error: ") + usb_strerror());
            }
            total_written += ret;
        }
    }

    // Send command and check response
    void send_cmd(const std::vector<uint8_t>& cmd) {
        write_bytes(cmd);
        auto resp = read_bytes(2);
        check(resp[0] == cmd[0], 
              "Command returned garbage (" + std::to_string(resp[0]) + 
              " != " + std::to_string(cmd[0]) + ")");
        if (resp[1] != 0) {
            throw CmdError(cmd, resp[1]);
        }
    }

    // Decode flux stream from device
    std::pair<std::vector<double>, std::vector<double>> decode_flux(const std::vector<uint8_t>& dat) {
        std::vector<double> flux;
        std::vector<double> index;
        
        if (dat.empty() || dat.back() != 0) {
            throw Fatal("Flux is not NUL-terminated");
        }

        size_t i = 0;
        size_t len = dat.size() - 1;  // Exclude trailing zero
        int64_t ticks = 0;
        int64_t ticks_since_index = 0;

        auto read_28bit = [&]() -> int {
            int val = (dat[i] & 0xFE) >> 1;
            val += (dat[i + 1] & 0xFE) << 6;
            val += (dat[i + 2] & 0xFE) << 13;
            val += (dat[i + 3] & 0xFE) << 20;
            i += 4;
            return val;
        };

        while (i < len) {
            uint8_t b = dat[i++];
            if (b == 255) {
                if (i >= len) {
                    throw Fatal("Unexpected end of flux");
                }
                uint8_t opcode = dat[i++];
                if (opcode == FluxOp::Index) {
                    if (i + 4 > len) {
                        throw Fatal("Unexpected end of flux");
                    }
                    int val = read_28bit();
                    index.push_back(static_cast<double>(ticks_since_index + ticks + val));
                    ticks_since_index = -(ticks + val);
                } else if (opcode == FluxOp::Space) {
                    if (i + 4 > len) {
                        throw Fatal("Unexpected end of flux");
                    }
                    ticks += read_28bit();
                } else {
                    throw Fatal("Bad opcode in flux stream (" + std::to_string(opcode) + ")");
                }
            } else {
                int val;
                if (b < 250) {
                    val = b;
                } else {
                    if (i >= len) {
                        throw Fatal("Unexpected end of flux");
                    }
                    val = 250 + (b - 250) * 255;
                    val += dat[i++] - 1;
                }
                ticks += val;
                flux.push_back(static_cast<double>(ticks));
                ticks_since_index += ticks;
                ticks = 0;
            }
        }

        return {flux, index};
    }

    // Encode flux stream for device
    std::vector<uint8_t> encode_flux(const std::vector<int>& flux) {
        int nfa_thresh = static_cast<int>(std::round(150e-6 * sample_freq));
        int nfa_period = static_cast<int>(std::round(1.25e-6 * sample_freq));
        
        std::vector<uint8_t> dat;

        auto write_28bit = [&](int x) {
            dat.push_back(1 | ((x << 1) & 255));
            dat.push_back(1 | ((x >> 6) & 255));
            dat.push_back(1 | ((x >> 13) & 255));
            dat.push_back(1 | ((x >> 20) & 255));
        };

        // Add dummy final flux value
        int dummy_flux = static_cast<int>(std::round(100e-6 * sample_freq));
        std::vector<int> flux_with_dummy = flux;
        flux_with_dummy.push_back(dummy_flux);

        for (int val : flux_with_dummy) {
            if (val == 0) {
                continue;
            } else if (val < 250) {
                dat.push_back(static_cast<uint8_t>(val));
            } else if (val > nfa_thresh) {
                dat.push_back(255);
                dat.push_back(FluxOp::Space);
                write_28bit(val);
                dat.push_back(255);
                dat.push_back(FluxOp::Astable);
                write_28bit(nfa_period);
            } else {
                int high = (val - 250) / 255;
                if (high < 5) {
                    dat.push_back(static_cast<uint8_t>(250 + high));
                    dat.push_back(static_cast<uint8_t>(1 + (val - 250) % 255));
                } else {
                    dat.push_back(255);
                    dat.push_back(FluxOp::Space);
                    write_28bit(val - 249);
                    dat.push_back(249);
                }
            }
        }
        dat.push_back(0);  // End of Stream
        return dat;
    }

    // Read track helper
    std::vector<uint8_t> read_track_raw(int revs, uint32_t ticks) {
        std::vector<uint8_t> cmd = {
            Cmd::ReadFlux, 8,
            static_cast<uint8_t>(ticks & 0xFF),
            static_cast<uint8_t>((ticks >> 8) & 0xFF),
            static_cast<uint8_t>((ticks >> 16) & 0xFF),
            static_cast<uint8_t>((ticks >> 24) & 0xFF),
            static_cast<uint8_t>((revs == 0) ? 0 : (revs + 1)),
            0
        };
        send_cmd(cmd);

        std::vector<uint8_t> dat;
        while (true) {
            auto chunk = read_bytes(1);
            dat.push_back(chunk[0]);
            if (chunk[0] == 0) break;
            
            // Read more available data (polling-style)
            // In libusb 0.1, we need to use bulk reads with timeout
            while (true) {
                char buf[64];
                int ret = usb_bulk_read(dev_handle_, endpoint_in_, buf, sizeof(buf), 100);
                if (ret <= 0) break;
                for (int j = 0; j < ret; ++j) {
                    dat.push_back(static_cast<uint8_t>(buf[j]));
                    if (static_cast<uint8_t>(buf[j]) == 0) break;
                }
                if (dat.back() == 0) break;
            }
            if (dat.back() == 0) break;
        }

        // Check flux status
        send_cmd({Cmd::GetFluxStatus, 2});
        return dat;
    }

public:
    // Firmware info
    uint8_t major;
    uint8_t minor;
    uint8_t max_cmd;
    uint32_t sample_freq;
    uint8_t hw_model;
    uint8_t hw_submodel;
    uint8_t usb_speed;
    uint8_t mcu_id;
    uint16_t mcu_mhz;
    uint16_t mcu_sram_kb;
    uint16_t usb_buf_kb;
    std::pair<int, int> version;
    bool update_mode;
    bool update_jumpered;
    bool update_needed;

    /**
     * @brief Construct GreaseweazleUnit from USB device handle
     * @param dev_handle libusb 0.1 device handle
     * @param endpoint_in Bulk IN endpoint
     * @param endpoint_out Bulk OUT endpoint
     * @param interface Interface number
     */
    GreaseweazleUnit(usb_dev_handle* dev_handle, int endpoint_in = 0x81, int endpoint_out = 0x02, int interface = 0)
        : dev_handle_(dev_handle)
        , endpoint_in_(endpoint_in)
        , endpoint_out_(endpoint_out)
        , interface_(interface)
    {
        reset();
        
        // Get firmware info
        send_cmd({Cmd::GetInfo, 3, GetInfo::Firmware});
        auto info = read_bytes(32);

        major = info[0];
        minor = info[1];
        uint8_t is_main_firmware = info[2];
        max_cmd = info[3];
        std::memcpy(&sample_freq, &info[4], sizeof(sample_freq));
        hw_model = info[8];
        hw_submodel = info[9];
        usb_speed = info[10];
        mcu_id = info[11];
        std::memcpy(&mcu_mhz, &info[12], sizeof(mcu_mhz));
        std::memcpy(&mcu_sram_kb, &info[14], sizeof(mcu_sram_kb));
        std::memcpy(&usb_buf_kb, &info[16], sizeof(usb_buf_kb));

        version = {major, minor};

        // Old firmware doesn't report HW type but runs on STM32F1 only
        if (hw_model == 0) {
            hw_model = 1;
        }

        // Check update mode
        update_mode = (is_main_firmware == 0);
        if (update_mode) {
            update_jumpered = (sample_freq & 1) != 0;
            sample_freq = 0;
            return;
        }

        // Check if update is needed
        update_needed = (version < EARLIEST_SUPPORTED_FIRMWARE);
    }

    ~GreaseweazleUnit() {
        if (dev_handle_) {
            usb_release_interface(dev_handle_, interface_);
            usb_close(dev_handle_);
        }
    }

    // Prevent copying
    GreaseweazleUnit(const GreaseweazleUnit&) = delete;
    GreaseweazleUnit& operator=(const GreaseweazleUnit&) = delete;

    // Allow moving
    GreaseweazleUnit(GreaseweazleUnit&& other) noexcept
        : dev_handle_(other.dev_handle_)
        , endpoint_in_(other.endpoint_in_)
        , endpoint_out_(other.endpoint_out_)
        , interface_(other.interface_)
    {
        other.dev_handle_ = nullptr;
    }

    /**
     * @brief Reset communications with Greaseweazle
     */
    void reset() {
        usb_clear_halt(dev_handle_, endpoint_in_);
        usb_clear_halt(dev_handle_, endpoint_out_);
    }

    /**
     * @brief Get current drive info
     */
    DriveInfo get_current_drive_info() {
        send_cmd({Cmd::GetInfo, 3, GetInfo::CurrentDrive});
        auto resp = read_bytes(32);
        return DriveInfo(resp.data());
    }

    /**
     * @brief Get parameters
     */
    std::vector<uint8_t> get_params(uint8_t idx, uint8_t nr) {
        send_cmd({Cmd::GetParams, 4, idx, nr});
        return read_bytes(nr);
    }

    /**
     * @brief Set parameters
     */
    void set_params(uint8_t idx, const std::vector<uint8_t>& dat) {
        std::vector<uint8_t> cmd = {Cmd::SetParams, static_cast<uint8_t>(3 + dat.size()), idx};
        cmd.insert(cmd.end(), dat.begin(), dat.end());
        send_cmd(cmd);
    }

    /**
     * @brief Seek to specified track
     */
    void seek(int cyl, int head) {
        std::vector<uint8_t> cmd;
        if (cyl >= -0x80 && cyl <= 0x7f) {
            cmd = {Cmd::Seek, 3, static_cast<uint8_t>(cyl)};
        } else if (cyl >= -0x8000 && cyl <= 0x7fff) {
            int16_t cyl16 = static_cast<int16_t>(cyl);
            cmd = {Cmd::Seek, 4, 
                   static_cast<uint8_t>(cyl16 & 0xFF),
                   static_cast<uint8_t>((cyl16 >> 8) & 0xFF)};
        } else {
            throw Fatal("Seek: Invalid cylinder " + std::to_string(cyl));
        }
        send_cmd(cmd);

        bool trk0 = !get_pin(26);
        if (cyl == 0 && !trk0) {
            // Handle flippy-modded drives
            try {
                auto info = get_current_drive_info();
                if (info.is_flippy) {
                    try {
                        send_cmd({Cmd::NoClickStep, 2});
                    } catch (...) {}
                }
            } catch (...) {}
            trk0 = !get_pin(26);
        }

        check(cyl < 0 || (cyl == 0) == trk0,
              std::string("Track0 signal ") + (trk0 ? "asserted" : "absent") +
              " after seek to cylinder " + std::to_string(cyl));
        
        send_cmd({Cmd::Head, 3, static_cast<uint8_t>(head)});
    }

    /**
     * @brief Set bus type
     */
    void set_bus_type(BusType type) {
        send_cmd({Cmd::SetBusType, 3, static_cast<uint8_t>(type)});
    }

    /**
     * @brief Set pin level
     */
    void set_pin(uint8_t pin, bool level) {
        send_cmd({Cmd::SetPin, 4, pin, static_cast<uint8_t>(level ? 1 : 0)});
    }

    /**
     * @brief Get pin level
     */
    bool get_pin(uint8_t pin) {
        send_cmd({Cmd::GetPin, 3, pin});
        auto resp = read_bytes(1);
        return resp[0] != 0;
    }

    /**
     * @brief Power-on reset
     */
    void power_on_reset() {
        send_cmd({Cmd::Reset, 2});
    }

    /**
     * @brief Select drive unit
     */
    void drive_select(uint8_t unit) {
        send_cmd({Cmd::Select, 3, unit});
    }

    /**
     * @brief Deselect current drive
     */
    void drive_deselect() {
        send_cmd({Cmd::Deselect, 2});
    }

    /**
     * @brief Set drive motor state
     */
    void drive_motor(uint8_t unit, bool state) {
        send_cmd({Cmd::Motor, 4, unit, static_cast<uint8_t>(state ? 1 : 0)});
    }

    /**
     * @brief Switch firmware mode
     */
    void switch_fw_mode(int mode) {
        send_cmd({Cmd::SwitchFwMode, 3, static_cast<uint8_t>(mode)});
    }

    /**
     * @brief Update main firmware
     */
    uint8_t update_main_firmware(const std::vector<uint8_t>& dat) {
        uint32_t len = static_cast<uint32_t>(dat.size());
        std::vector<uint8_t> cmd = {
            Cmd::Update, 6,
            static_cast<uint8_t>(len & 0xFF),
            static_cast<uint8_t>((len >> 8) & 0xFF),
            static_cast<uint8_t>((len >> 16) & 0xFF),
            static_cast<uint8_t>((len >> 24) & 0xFF)
        };
        send_cmd(cmd);
        write_bytes(dat);
        auto resp = read_bytes(1);
        return resp[0];
    }

    /**
     * @brief Update bootloader
     */
    uint8_t update_bootloader(const std::vector<uint8_t>& dat) {
        uint32_t len = static_cast<uint32_t>(dat.size());
        uint32_t magic = 0xdeafbee3;
        std::vector<uint8_t> cmd = {
            Cmd::Update, 10,
            static_cast<uint8_t>(len & 0xFF),
            static_cast<uint8_t>((len >> 8) & 0xFF),
            static_cast<uint8_t>((len >> 16) & 0xFF),
            static_cast<uint8_t>((len >> 24) & 0xFF),
            static_cast<uint8_t>(magic & 0xFF),
            static_cast<uint8_t>((magic >> 8) & 0xFF),
            static_cast<uint8_t>((magic >> 16) & 0xFF),
            static_cast<uint8_t>((magic >> 24) & 0xFF)
        };
        send_cmd(cmd);
        write_bytes(dat);
        auto resp = read_bytes(1);
        return resp[0];
    }

    /**
     * @brief Read track and decode flux
     */
    Flux read_track(int revs, uint32_t ticks = 0, int nr_retries = 5) {
        int retry = 0;
        std::vector<uint8_t> dat;
        
        while (true) {
            try {
                dat = read_track_raw(revs, ticks);
                break;
            } catch (CmdError& error) {
                if (error.code == Ack::FluxOverflow && retry < nr_retries) {
                    retry++;
                    continue;
                }
                throw;
            }
        }

        auto [flux_list, index_list] = decode_flux(dat);
        return Flux(index_list, flux_list, static_cast<double>(sample_freq), false);
    }

    /**
     * @brief Write track from flux list
     */
    void write_track(const std::vector<int>& flux_list, bool terminate_at_index,
                     bool cue_at_index = true, int nr_retries = 5,
                     uint32_t hard_sector_ticks = 0) {
        auto dat = encode_flux(flux_list);

        int retry = 0;
        while (true) {
            try {
                std::vector<uint8_t> cmd;
                if (hard_sector_ticks != 0) {
                    cmd = {
                        Cmd::WriteFlux, 8,
                        static_cast<uint8_t>(cue_at_index ? 1 : 0),
                        static_cast<uint8_t>(terminate_at_index ? 1 : 0),
                        static_cast<uint8_t>(hard_sector_ticks & 0xFF),
                        static_cast<uint8_t>((hard_sector_ticks >> 8) & 0xFF),
                        static_cast<uint8_t>((hard_sector_ticks >> 16) & 0xFF),
                        static_cast<uint8_t>((hard_sector_ticks >> 24) & 0xFF)
                    };
                } else {
                    cmd = {
                        Cmd::WriteFlux, 4,
                        static_cast<uint8_t>(cue_at_index ? 1 : 0),
                        static_cast<uint8_t>(terminate_at_index ? 1 : 0)
                    };
                }
                send_cmd(cmd);
                write_bytes(dat.data(), dat.size());
                read_bytes(1);  // Sync
                send_cmd({Cmd::GetFluxStatus, 2});
                break;
            } catch (CmdError& error) {
                if (error.code == Ack::FluxUnderflow && retry < nr_retries) {
                    retry++;
                    continue;
                }
                throw;
            }
        }
    }

    /**
     * @brief Erase current track
     */
    void erase_track(uint32_t ticks) {
        std::vector<uint8_t> cmd = {
            Cmd::EraseFlux, 6,
            static_cast<uint8_t>(ticks & 0xFF),
            static_cast<uint8_t>((ticks >> 8) & 0xFF),
            static_cast<uint8_t>((ticks >> 16) & 0xFF),
            static_cast<uint8_t>((ticks >> 24) & 0xFF)
        };
        send_cmd(cmd);
        read_bytes(1);  // Sync
        send_cmd({Cmd::GetFluxStatus, 2});
    }

    /**
     * @brief Source bytes for bandwidth testing
     */
    std::vector<uint8_t> source_bytes(uint32_t nr, uint32_t seed) {
        std::vector<uint8_t> cmd = {
            Cmd::SourceBytes, 10,
            static_cast<uint8_t>(nr & 0xFF),
            static_cast<uint8_t>((nr >> 8) & 0xFF),
            static_cast<uint8_t>((nr >> 16) & 0xFF),
            static_cast<uint8_t>((nr >> 24) & 0xFF),
            static_cast<uint8_t>(seed & 0xFF),
            static_cast<uint8_t>((seed >> 8) & 0xFF),
            static_cast<uint8_t>((seed >> 16) & 0xFF),
            static_cast<uint8_t>((seed >> 24) & 0xFF)
        };
        send_cmd(cmd);
        return read_bytes(nr);
    }

    /**
     * @brief Sink bytes for bandwidth testing
     */
    uint8_t sink_bytes(const std::vector<uint8_t>& dat, uint32_t seed) {
        uint32_t len = static_cast<uint32_t>(dat.size());
        std::vector<uint8_t> cmd = {
            Cmd::SinkBytes, 10,
            static_cast<uint8_t>(len & 0xFF),
            static_cast<uint8_t>((len >> 8) & 0xFF),
            static_cast<uint8_t>((len >> 16) & 0xFF),
            static_cast<uint8_t>((len >> 24) & 0xFF),
            static_cast<uint8_t>(seed & 0xFF),
            static_cast<uint8_t>((seed >> 8) & 0xFF),
            static_cast<uint8_t>((seed >> 16) & 0xFF),
            static_cast<uint8_t>((seed >> 24) & 0xFF)
        };
        send_cmd(cmd);
        write_bytes(dat);
        auto resp = read_bytes(1);
        return resp[0];
    }

    /**
     * @brief Get bandwidth statistics
     */
    std::pair<double, double> bw_stats() {
        send_cmd({Cmd::GetInfo, 3, GetInfo::BandwidthStats});
        auto resp = read_bytes(32);
        
        uint32_t min_bytes, min_usecs, max_bytes, max_usecs;
        std::memcpy(&min_bytes, resp.data(), sizeof(min_bytes));
        std::memcpy(&min_usecs, resp.data() + 4, sizeof(min_usecs));
        std::memcpy(&max_bytes, resp.data() + 8, sizeof(max_bytes));
        std::memcpy(&max_usecs, resp.data() + 12, sizeof(max_usecs));

        double min_bw = (8.0 * min_bytes) / min_usecs;
        double max_bw = (8.0 * max_bytes) / max_usecs;
        return {min_bw, max_bw};
    }

    /**
     * @brief Find and open a Greaseweazle device
     * @return Unique pointer to GreaseweazleUnit, or nullptr if not found
     */
    static std::unique_ptr<GreaseweazleUnit> open() {
        usb_init();
        usb_find_busses();
        usb_find_devices();

        // Greaseweazle USB IDs
        constexpr uint16_t GW_VID = 0x1209;
        constexpr uint16_t GW_PID = 0x4d69;  // Official PID
        constexpr uint16_t GW_TEST_PID = 0x0001;  // Test PID

        for (struct usb_bus* bus = usb_get_busses(); bus; bus = bus->next) {
            for (struct usb_device* dev = bus->devices; dev; dev = dev->next) {
                if ((dev->descriptor.idVendor == GW_VID && 
                     (dev->descriptor.idProduct == GW_PID || 
                      dev->descriptor.idProduct == GW_TEST_PID))) {
                    
                    usb_dev_handle* handle = usb_open(dev);
                    if (!handle) continue;

                    // Find bulk endpoints
                    int endpoint_in = 0x81;
                    int endpoint_out = 0x02;
                    int interface_num = 0;

                    if (dev->config) {
                        for (int i = 0; i < dev->config->bNumInterfaces; i++) {
                            for (int j = 0; j < dev->config->interface[i].num_altsetting; j++) {
                                usb_interface_descriptor* iface = 
                                    &dev->config->interface[i].altsetting[j];
                                for (int k = 0; k < iface->bNumEndpoints; k++) {
                                    usb_endpoint_descriptor* ep = &iface->endpoint[k];
                                    if ((ep->bmAttributes & USB_ENDPOINT_TYPE_MASK) == 
                                        USB_ENDPOINT_TYPE_BULK) {
                                        if (ep->bEndpointAddress & USB_ENDPOINT_DIR_MASK) {
                                            endpoint_in = ep->bEndpointAddress;
                                        } else {
                                            endpoint_out = ep->bEndpointAddress;
                                        }
                                    }
                                }
                                interface_num = iface->bInterfaceNumber;
                            }
                        }
                    }

                    // Claim interface
                    #ifdef LIBUSB_HAS_DETACH_KERNEL_DRIVER_NP
                    usb_detach_kernel_driver_np(handle, interface_num);
                    #endif
                    
                    if (usb_claim_interface(handle, interface_num) < 0) {
                        usb_close(handle);
                        continue;
                    }

                    try {
                        return std::make_unique<GreaseweazleUnit>(handle, endpoint_in, 
                                                      endpoint_out, interface_num);
                    } catch (...) {
                        usb_release_interface(handle, interface_num);
                        usb_close(handle);
                        throw;
                    }
                }
            }
        }

        return nullptr;
    }
};

} // namespace greaseweazle

#endif // GREASEWEAZLE_USB_HPP
