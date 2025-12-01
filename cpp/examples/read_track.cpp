/*
 * read_track.cpp
 *
 * Example: Read flux data from a track
 *
 * C++ port using libusb 0.1
 * Original Python code written & released by Keir Fraser <keir.xen@gmail.com>
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include "greaseweazle.hpp"

using namespace greaseweazle;

void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  -c <cyl>    Cylinder to read (default: 0)" << std::endl;
    std::cerr << "  -h <head>   Head to read (default: 0)" << std::endl;
    std::cerr << "  -r <revs>   Number of revolutions (default: 3)" << std::endl;
    std::cerr << "  -d <drive>  Drive number (default: 0)" << std::endl;
    std::cerr << "  -o <file>   Output filename for raw flux data" << std::endl;
}

int main(int argc, char* argv[]) {
    int cylinder = 0;
    int head = 0;
    int revs = 3;
    int drive = 0;
    std::string output_file;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-c" && i + 1 < argc) {
            cylinder = std::atoi(argv[++i]);
        } else if (arg == "-h" && i + 1 < argc) {
            head = std::atoi(argv[++i]);
        } else if (arg == "-r" && i + 1 < argc) {
            revs = std::atoi(argv[++i]);
        } else if (arg == "-d" && i + 1 < argc) {
            drive = std::atoi(argv[++i]);
        } else if (arg == "-o" && i + 1 < argc) {
            output_file = argv[++i];
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    std::cout << "Greaseweazle Read Track (C++ libusb 0.1 port)" << std::endl;
    std::cout << "Reading cylinder " << cylinder << ", head " << head 
              << ", " << revs << " revolutions" << std::endl;

    try {
        auto usb = GreaseweazleUnit::open();
        
        if (!usb) {
            std::cerr << "Error: Greaseweazle device not found" << std::endl;
            return 1;
        }

        if (usb->update_mode) {
            std::cerr << "Error: Device is in update mode" << std::endl;
            return 1;
        }

        std::cout << "Device found!" << std::endl;
        std::cout << "Firmware: " << usb->version.first << "." 
                  << usb->version.second << std::endl;
        std::cout << "Sample frequency: " << std::fixed << std::setprecision(2)
                  << (usb->sample_freq / 1e6) << " MHz" << std::endl;

        // Set bus type (Shugart for drive 0-3)
        usb->set_bus_type(BusType::Shugart);

        // Select drive
        usb->drive_select(static_cast<uint8_t>(drive));

        // Turn on motor
        usb->drive_motor(static_cast<uint8_t>(drive), true);

        // Wait for motor to spin up
        std::cout << "Motor on, waiting for spin-up..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Seek to track
        std::cout << "Seeking to cylinder " << cylinder << ", head " << head << std::endl;
        usb->seek(cylinder, head);

        // Read flux
        std::cout << "Reading flux data..." << std::endl;
        Flux flux = usb->read_track(revs);

        // Print summary
        std::cout << std::endl;
        std::cout << "Read completed:" << std::endl;
        std::cout << "  Total flux samples: " << flux.list.size() << std::endl;
        std::cout << "  Index pulses: " << flux.index_list.size() << std::endl;
        std::cout << "  Sample frequency: " << std::fixed << std::setprecision(2) 
                  << (flux.sample_freq / 1e6) << " MHz" << std::endl;

        for (size_t i = 0; i < flux.index_list.size(); ++i) {
            double rev_time_ms = flux.index_list[i] * 1000.0 / flux.sample_freq;
            double rpm = 60000.0 / rev_time_ms;
            std::cout << "  Revolution " << i << ": " << std::fixed 
                      << std::setprecision(2) << rev_time_ms << " ms (" 
                      << rpm << " RPM)" << std::endl;
        }

        // Save raw flux data if output file specified
        if (!output_file.empty()) {
            std::ofstream ofs(output_file, std::ios::binary);
            if (ofs) {
                // Write simple binary format:
                // - uint32_t: sample frequency
                // - uint32_t: number of index entries
                // - double[]: index list
                // - uint32_t: number of flux entries
                // - double[]: flux list
                uint32_t sf = usb->sample_freq;
                uint32_t num_idx = static_cast<uint32_t>(flux.index_list.size());
                uint32_t num_flux = static_cast<uint32_t>(flux.list.size());

                ofs.write(reinterpret_cast<const char*>(&sf), sizeof(sf));
                ofs.write(reinterpret_cast<const char*>(&num_idx), sizeof(num_idx));
                for (double idx : flux.index_list) {
                    ofs.write(reinterpret_cast<const char*>(&idx), sizeof(idx));
                }
                ofs.write(reinterpret_cast<const char*>(&num_flux), sizeof(num_flux));
                for (double f : flux.list) {
                    ofs.write(reinterpret_cast<const char*>(&f), sizeof(f));
                }
                ofs.close();
                std::cout << "Flux data saved to: " << output_file << std::endl;
            } else {
                std::cerr << "Error: Could not open output file" << std::endl;
            }
        }

        // Turn off motor and deselect
        usb->drive_motor(static_cast<uint8_t>(drive), false);
        usb->drive_deselect();

        std::cout << std::endl;
        std::cout << "Done!" << std::endl;

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
