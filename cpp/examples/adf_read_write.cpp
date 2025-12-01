/*
 * adf_read_write.cpp
 *
 * Example: Read and write ADF files to/from floppy disks
 *
 * This example demonstrates how to:
 * - Read a floppy disk to an ADF file (DiskToADF)
 * - Write an ADF file to a floppy disk (ADFToDisk)
 *
 * Based on the ADFWriter class from ArduinoFloppyReader/wafflereader.
 *
 * C++ implementation using Greaseweazle C++ library with libusb 0.1
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include "greaseweazle.hpp"
#include "greaseweazle/amiga_adf.hpp"

using namespace greaseweazle;
using namespace greaseweazle::amiga;

/**
 * @brief Print usage information
 */
void print_usage(const char* prog) {
    std::cerr << "Amiga ADF Floppy Tool (Greaseweazle C++ port)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: " << prog << " <command> [options]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Commands:" << std::endl;
    std::cerr << "  read <output.adf>    Read disk to ADF file (DiskToADF)" << std::endl;
    std::cerr << "  write <input.adf>    Write ADF file to disk (ADFToDisk)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  -h, --hd             Use high-density mode (1.76MB)" << std::endl;
    std::cerr << "  -t, --tracks <n>     Number of tracks to read (default: 80)" << std::endl;
    std::cerr << "  -v, --verify         Verify after writing (default: on for write)" << std::endl;
    std::cerr << "  -n, --no-verify      Skip verification after writing" << std::endl;
    std::cerr << "  -d, --drive <n>      Drive number (default: 0)" << std::endl;
    std::cerr << "  -q, --quiet          Suppress progress output" << std::endl;
    std::cerr << "  --help               Show this help message" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Examples:" << std::endl;
    std::cerr << "  " << prog << " read mydisk.adf          Read DD disk to ADF" << std::endl;
    std::cerr << "  " << prog << " read -h mydisk.adf       Read HD disk to ADF" << std::endl;
    std::cerr << "  " << prog << " write mydisk.adf         Write ADF to DD disk" << std::endl;
    std::cerr << "  " << prog << " write -h mydisk.adf      Write ADF to HD disk" << std::endl;
}

/**
 * @brief Progress callback for read operations
 */
WriteResponse read_progress_callback(int track, DiskSurface side, int retry,
                                     int found, int bad, int total,
                                     CallbackOperation op) {
    const char* sideStr = (side == DiskSurface::Upper) ? "Upper" : "Lower";
    
    switch (op) {
        case CallbackOperation::Starting:
            std::cout << "Starting disk read..." << std::endl;
            break;
        case CallbackOperation::Reading:
            std::cout << "\rTrack " << std::setw(2) << track 
                      << " [" << sideStr << "]: "
                      << std::setw(2) << found << "/" << total << " sectors";
            std::cout.flush();
            break;
        case CallbackOperation::RetryReading:
            std::cout << "\rTrack " << std::setw(2) << track 
                      << " [" << sideStr << "]: "
                      << std::setw(2) << found << "/" << total << " sectors"
                      << " (retry " << retry << ", " << bad << " bad)";
            std::cout.flush();
            break;
        default:
            break;
    }
    
    return WriteResponse::Continue;
}

/**
 * @brief Progress callback for write operations
 */
WriteResponse write_progress_callback(int track, DiskSurface side, bool verifyError,
                                      CallbackOperation op) {
    const char* sideStr = (side == DiskSurface::Upper) ? "Upper" : "Lower";
    
    switch (op) {
        case CallbackOperation::Starting:
            std::cout << "Starting disk write..." << std::endl;
            break;
        case CallbackOperation::ReadingFile:
            // Quiet during file read
            break;
        case CallbackOperation::Writing:
            std::cout << "\rTrack " << std::setw(2) << track 
                      << " [" << sideStr << "]: Writing...";
            std::cout.flush();
            break;
        case CallbackOperation::RetryWriting:
            std::cout << "\rTrack " << std::setw(2) << track 
                      << " [" << sideStr << "]: Retrying...";
            std::cout.flush();
            break;
        case CallbackOperation::Verifying:
            std::cout << "\rTrack " << std::setw(2) << track 
                      << " [" << sideStr << "]: Verifying...";
            std::cout.flush();
            break;
        case CallbackOperation::ReVerifying:
            if (verifyError) {
                std::cout << "\rTrack " << std::setw(2) << track 
                          << " [" << sideStr << "]: Verify failed, retrying...";
                std::cout.flush();
            }
            break;
        default:
            break;
    }
    
    return WriteResponse::Continue;
}

/**
 * @brief Read disk to ADF file
 */
int do_read(const std::string& outputFile, bool isHD, int numTracks, 
            int drive, bool quiet) {
    std::cout << "Reading disk to: " << outputFile << std::endl;
    std::cout << "Mode: " << (isHD ? "High Density (HD)" : "Double Density (DD)") << std::endl;
    std::cout << "Tracks: " << numTracks << std::endl;
    std::cout << std::endl;
    
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
        std::cout << std::endl;
        
        ADFReadCallback callback = quiet ? nullptr : read_progress_callback;
        
        ADFResult result = disk_to_adf(*usb, outputFile, isHD, numTracks, callback);
        
        std::cout << std::endl;
        std::cout << "Result: " << adf_result_string(result) << std::endl;
        
        return (result == ADFResult::Complete) ? 0 : 1;
        
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

/**
 * @brief Write ADF file to disk
 */
int do_write(const std::string& inputFile, bool isHD, bool verify,
             int drive, bool quiet) {
    std::cout << "Writing to disk: " << inputFile << std::endl;
    std::cout << "Mode: " << (isHD ? "High Density (HD)" : "Double Density (DD)") << std::endl;
    std::cout << "Verify: " << (verify ? "Yes" : "No") << std::endl;
    std::cout << std::endl;
    
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
        std::cout << std::endl;
        
        ADFWriteCallback callback = quiet ? nullptr : write_progress_callback;
        
        ADFResult result = adf_to_disk(*usb, inputFile, isHD, verify, callback);
        
        std::cout << std::endl;
        std::cout << "Result: " << adf_result_string(result) << std::endl;
        
        return (result == ADFResult::Complete) ? 0 : 1;
        
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

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    std::string command;
    std::string filename;
    bool isHD = false;
    int numTracks = 80;
    bool verify = true;
    int drive = 0;
    bool quiet = false;
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "-h" || arg == "--hd") {
            isHD = true;
        } else if (arg == "-v" || arg == "--verify") {
            verify = true;
        } else if (arg == "-n" || arg == "--no-verify") {
            verify = false;
        } else if (arg == "-q" || arg == "--quiet") {
            quiet = true;
        } else if ((arg == "-t" || arg == "--tracks") && i + 1 < argc) {
            numTracks = std::atoi(argv[++i]);
            if (numTracks < 1 || numTracks > 84) {
                std::cerr << "Error: Invalid track count (1-84)" << std::endl;
                return 1;
            }
        } else if ((arg == "-d" || arg == "--drive") && i + 1 < argc) {
            drive = std::atoi(argv[++i]);
            if (drive < 0 || drive > 3) {
                std::cerr << "Error: Invalid drive number (0-3)" << std::endl;
                return 1;
            }
        } else if (command.empty()) {
            command = arg;
        } else if (filename.empty()) {
            filename = arg;
        } else {
            std::cerr << "Error: Unknown argument: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    
    // Validate
    if (command.empty()) {
        std::cerr << "Error: No command specified" << std::endl;
        print_usage(argv[0]);
        return 1;
    }
    
    if (filename.empty()) {
        std::cerr << "Error: No filename specified" << std::endl;
        print_usage(argv[0]);
        return 1;
    }
    
    // Execute command
    if (command == "read") {
        return do_read(filename, isHD, numTracks, drive, quiet);
    } else if (command == "write") {
        return do_write(filename, isHD, verify, drive, quiet);
    } else {
        std::cerr << "Error: Unknown command: " << command << std::endl;
        print_usage(argv[0]);
        return 1;
    }
}
