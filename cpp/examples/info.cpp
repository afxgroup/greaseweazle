/*
 * info.cpp
 *
 * Example: Display information about the Greaseweazle setup
 *
 * C++ port using libusb 0.1
 * Original Python code written & released by Keir Fraser <keir.xen@gmail.com>
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#include <iostream>
#include <iomanip>
#include <map>
#include "greaseweazle.hpp"

using namespace greaseweazle;

// Model ID mappings
const std::map<std::pair<int, int>, std::string> model_id = {
    {{1, 0}, "F1"},
    {{1, 1}, "F1 Plus"},
    {{1, 2}, "F1 Plus (Unbuffered)"},
    {{4, 0}, "V4"},
    {{4, 1}, "V4 Slim"},
    {{4, 2}, "V4.1"},
    {{7, 0}, "F7 v1"},
    {{7, 1}, "F7 Plus (Ant Goffart, v1)"},
    {{7, 2}, "F7 Lightning"},
    {{7, 3}, "F7 v2"},
    {{7, 4}, "F7 Plus (Ant Goffart, v2)"},
    {{7, 5}, "F7 Lightning Plus"},
    {{7, 6}, "F7 Slim"},
    {{7, 7}, "F7 v3 \"Thunderbolt\""},
    {{8, 0}, "Adafruit Floppy Generic"}
};

const std::map<int, std::string> speed_id = {
    {0, "Full Speed (12 Mbit/s)"},
    {1, "High Speed (480 Mbit/s)"}
};

const std::map<int, std::string> mcu_id_map = {
    {2, "AT32F403"},
    {7, "AT32F403A"},
    {5, "AT32F415"}
};

void print_info_line(const std::string& name, const std::string& value, int tab = 0) {
    std::cout << std::string(tab, ' ') 
              << std::left << std::setw(12 - tab) << (name + ":") 
              << value << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Greaseweazle Info (C++ libusb 0.1 port)" << std::endl;
    std::cout << std::endl;

    try {
        auto usb = Unit::open();
        
        if (!usb) {
            std::cout << "Device:" << std::endl;
            std::cout << "  Not found" << std::endl;
            return 1;
        }

        std::cout << "Device:" << std::endl;

        // Get model name
        std::string model;
        auto it = model_id.find({usb->hw_model, usb->hw_submodel});
        if (it != model_id.end()) {
            model = it->second;
            if (usb->hw_model != 8) {
                model = "Greaseweazle " + model;
            }
        } else {
            std::ostringstream ss;
            ss << "Unknown (0x" << std::hex << std::setfill('0') 
               << std::setw(2) << static_cast<int>(usb->hw_model)
               << std::setw(2) << static_cast<int>(usb->hw_submodel) << ")";
            model = ss.str();
        }
        print_info_line("Model", model, 2);

        // MCU info
        std::vector<std::string> mcu_strs;
        auto mcu_it = mcu_id_map.find(usb->mcu_id);
        if (mcu_it != mcu_id_map.end()) {
            mcu_strs.push_back(mcu_it->second);
        } else if (usb->mcu_id != 0) {
            std::ostringstream ss;
            ss << "Unknown (0x" << std::hex << std::setw(2) 
               << std::setfill('0') << static_cast<int>(usb->mcu_id) << ")";
            mcu_strs.push_back(ss.str());
        }
        if (usb->mcu_mhz) {
            mcu_strs.push_back(std::to_string(usb->mcu_mhz) + "MHz");
        }
        if (usb->mcu_sram_kb) {
            mcu_strs.push_back(std::to_string(usb->mcu_sram_kb) + "kB SRAM");
        }
        if (!mcu_strs.empty()) {
            std::string mcu_info;
            for (size_t i = 0; i < mcu_strs.size(); ++i) {
                if (i > 0) mcu_info += ", ";
                mcu_info += mcu_strs[i];
            }
            print_info_line("MCU", mcu_info, 2);
        }

        // Firmware version
        std::ostringstream fwver;
        fwver << usb->version.first << "." << usb->version.second;
        if (usb->update_mode) {
            fwver << " (Bootloader)";
        }
        print_info_line("Firmware", fwver.str(), 2);

        // USB speed
        std::vector<std::string> usb_strs;
        auto speed_it = speed_id.find(usb->usb_speed);
        if (speed_it != speed_id.end()) {
            usb_strs.push_back(speed_it->second);
        } else {
            std::ostringstream ss;
            ss << "Unknown (0x" << std::hex << std::setw(2) 
               << std::setfill('0') << static_cast<int>(usb->usb_speed) << ")";
            usb_strs.push_back(ss.str());
        }
        if (usb->usb_buf_kb) {
            usb_strs.push_back(std::to_string(usb->usb_buf_kb) + "kB Buffer");
        }
        if (!usb_strs.empty()) {
            std::string usb_info;
            for (size_t i = 0; i < usb_strs.size(); ++i) {
                if (i > 0) usb_info += ", ";
                usb_info += usb_strs[i];
            }
            print_info_line("USB", usb_info, 2);
        }

        // Sample frequency
        if (!usb->update_mode) {
            std::ostringstream freq_ss;
            freq_ss << std::fixed << std::setprecision(2) 
                    << (usb->sample_freq / 1e6) << " MHz";
            print_info_line("Sample Freq", freq_ss.str(), 2);
        }

        std::cout << std::endl;
        std::cout << "Device found and initialized successfully!" << std::endl;

        return 0;

    } catch (const Fatal& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    } catch (const CmdError& e) {
        std::cerr << "Command error: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
