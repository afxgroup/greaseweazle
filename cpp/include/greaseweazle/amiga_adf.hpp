/*
 * greaseweazle/amiga_adf.hpp
 *
 * Amiga ADF file read/write support.
 * 
 * Implements reading floppy disks to ADF files and writing ADF files to disks.
 * Based on the ADFWriter class from ArduinoFloppyReader/wafflereader.
 *
 * MFM decoding algorithm and sector format information from:
 * - Laurent Clévy: http://lclevy.free.fr/adflib/adf_info.html
 * - Keith Monahan: https://www.techtravels.org/tag/mfm/
 *
 * C++ implementation using Greaseweazle C++ library
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#ifndef GREASEWEAZLE_AMIGA_ADF_HPP
#define GREASEWEAZLE_AMIGA_ADF_HPP

#include <cstdint>
#include <cstring>
#include <chrono>
#include <thread>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

#include "usb.hpp"
#include "flux.hpp"
#include "error.hpp"

namespace greaseweazle {
namespace amiga {

// Constants for Amiga disk format
constexpr uint16_t AMIGA_WORD_SYNC = 0x4489;        // MFM sync word
constexpr uint32_t MFM_MASK = 0x55555555;           // MFM decoding mask

// Double Density (DD) disk constants
constexpr size_t NUM_SECTORS_PER_TRACK_DD = 11;     // 11 sectors per track for DD
constexpr size_t SECTOR_BYTES = 512;                // 512 bytes per sector
constexpr size_t RAW_SECTOR_SIZE = 1088;            // MFM encoded sector size
constexpr size_t RAW_TRACKDATA_LENGTH_DD = 12800;   // Raw track data length for DD
constexpr size_t ADF_TRACK_SIZE_DD = SECTOR_BYTES * NUM_SECTORS_PER_TRACK_DD;

// High Density (HD) disk constants  
constexpr size_t NUM_SECTORS_PER_TRACK_HD = 22;     // 22 sectors per track for HD
constexpr size_t RAW_TRACKDATA_LENGTH_HD = 25600;   // Raw track data length for HD
constexpr size_t ADF_TRACK_SIZE_HD = SECTOR_BYTES * NUM_SECTORS_PER_TRACK_HD;

// Standard number of tracks
constexpr size_t NUM_TRACKS_STANDARD = 80;
constexpr size_t NUM_SIDES = 2;

// Type definitions
using RawEncodedSector = uint8_t[RAW_SECTOR_SIZE];
using RawDecodedSector = uint8_t[SECTOR_BYTES];
using RawDecodedTrackDD = RawDecodedSector[NUM_SECTORS_PER_TRACK_DD];
using RawDecodedTrackHD = RawDecodedSector[NUM_SECTORS_PER_TRACK_HD];
using RawMFMData = uint8_t[SECTOR_BYTES * 2];
using RawTrackDataDD = uint8_t[RAW_TRACKDATA_LENGTH_DD];
using RawTrackDataHD = uint8_t[RAW_TRACKDATA_LENGTH_HD];

/**
 * @brief Disk surface enumeration
 */
enum class DiskSurface {
    Lower = 0,  // Side 0
    Upper = 1   // Side 1
};

/**
 * @brief Result codes for ADF operations
 */
enum class ADFResult {
    Complete = 0,
    CompletedWithErrors,
    Aborted,
    FileError,
    FileIOError,
    DriveError,
    DiskWriteProtected,
    MediaSizeMismatch,
    ExtendedADFNotSupported
};

/**
 * @brief Callback operation types
 */
enum class CallbackOperation {
    Starting,
    ReadingFile,
    Writing,
    RetryWriting,
    Verifying,
    ReVerifying,
    Reading,
    RetryReading
};

/**
 * @brief Write response from callback
 */
enum class WriteResponse {
    Continue,
    Retry,
    Abort,
    SkipBadChecksums
};

/**
 * @brief Structure to hold decoded sector information
 */
struct DecodedSector {
    uint8_t trackFormat;        // Should be 0xFF for Amiga
    uint8_t trackNumber;        // Track number (track * 2 + side)
    uint8_t sectorNumber;       // Sector number (0 to 10/21)
    uint8_t sectorsRemaining;   // Sectors remaining until gap

    uint32_t sectorLabel[4];    // OS Recovery Data (ignored)

    uint32_t headerChecksum;
    uint32_t dataChecksum;
    uint32_t headerChecksumCalculated;
    uint32_t dataChecksumCalculated;

    RawDecodedSector data;      // Decoded sector data
    RawMFMData rawSector;       // Raw MFM data for analysis
};

/**
 * @brief Structure to hold decoded track information
 */
struct DecodedTrack {
    std::vector<DecodedSector> validSectors;
    std::vector<DecodedSector> invalidSectors[NUM_SECTORS_PER_TRACK_HD];
};

/**
 * @brief Full disk track structure for DD (including gap padding)
 */
#pragma pack(push, 1)
struct FullDiskTrackDD {
    uint8_t filler1[1654];      // Padding at start (0xAA)
    RawEncodedSector sectors[NUM_SECTORS_PER_TRACK_DD];
    uint8_t filler2[8];         // Trailing gap
};

/**
 * @brief Full disk track structure for HD (including gap padding)
 */
struct FullDiskTrackHD {
    uint8_t filler1[1654];      // Padding at start (0xAA)
    RawEncodedSector sectors[NUM_SECTORS_PER_TRACK_HD];
    uint8_t filler2[8];         // Trailing gap
};
#pragma pack(pop)

/**
 * @brief MFM data decoder
 * 
 * Decodes MFM coded data buffer to decoded data buffer
 * @param input MFM coded data buffer (size == 2*data_size)
 * @param output Decoded data buffer (size == data_size)
 * @param data_size Size of decoded data
 * @return Checksum calculated over the data
 */
inline uint32_t decode_mfm_data(const uint32_t* input, uint32_t* output, size_t data_size) {
    uint32_t chksum = 0;
    
    for (size_t count = 0; count < data_size / 4; count++) {
        uint32_t odd_bits = input[count];
        const uint8_t* input_bytes = reinterpret_cast<const uint8_t*>(input);
        uint32_t even_bits;
        std::memcpy(&even_bits, input_bytes + data_size + (count * 4), sizeof(even_bits));
        
        chksum ^= odd_bits;
        chksum ^= even_bits;
        
        output[count] = ((even_bits & MFM_MASK) | ((odd_bits & MFM_MASK) << 1));
    }
    
    return chksum & MFM_MASK;
}

/**
 * @brief MFM data encoder (part 1)
 * 
 * Encodes raw data to MFM format (writes data bits only, not clock bits)
 * @param input Raw data buffer (size == data_size)
 * @param output MFM encoded buffer (size == data_size*2)
 * @param data_size Size of input data
 * @return Checksum calculated over the data
 */
inline uint32_t encode_mfm_data_part1(const uint32_t* input, uint32_t* output, size_t data_size) {
    uint32_t chksum = 0;
    
    uint32_t* outputOdd = output;
    uint8_t* output_bytes = reinterpret_cast<uint8_t*>(output);
    uint32_t* outputEven = reinterpret_cast<uint32_t*>(output_bytes + data_size);
    
    // Split odd and even bits
    for (size_t count = 0; count < data_size / 4; count++) {
        outputEven[count] = input[count] & MFM_MASK;
        outputOdd[count] = (input[count] >> 1) & MFM_MASK;
    }
    
    // Calculate checksum
    for (size_t count = 0; count < (data_size / 4) * 2; count++) {
        chksum ^= output[count];
    }
    
    return chksum & MFM_MASK;
}

/**
 * @brief Align sector data to byte boundary
 * 
 * Copies data from track buffer aligning it to the start of a byte
 * @param inTrack Input track data
 * @param dataLength Length of track data
 * @param byteStart Starting byte position
 * @param bitStart Starting bit position
 * @param outSector Output aligned sector
 */
inline void align_sector_to_byte(const uint8_t* inTrack, size_t dataLength, 
                                  int byteStart, int bitStart, RawEncodedSector& outSector) {
    uint8_t byteOut = 0;
    size_t byteOutPosition = 0;
    int counter = 0;
    
    // Position is last bit of sync, go to next bit
    bitStart--;
    if (bitStart < 0) {
        bitStart = 7;
        byteStart++;
    }
    byteStart -= 8;  // Wind back 8 bytes
    
    for (;;) {
        for (int bitCounter = bitStart; bitCounter >= 0; bitCounter--) {
            byteOut <<= 1;
            if (inTrack[byteStart % dataLength] & (1 << bitCounter)) {
                byteOut |= 1;
            }
            
            if (++counter >= 8) {
                outSector[byteOutPosition] = byteOut;
                byteOutPosition++;
                if (byteOutPosition >= RAW_SECTOR_SIZE) return;
                counter = 0;
            }
        }
        
        byteStart++;
        bitStart = 7;
    }
}

/**
 * @brief Decode a sector from raw MFM data
 * 
 * @param rawSector Raw MFM encoded sector
 * @param trackNumber Physical track number
 * @param isHD Whether this is a high-density disk
 * @param surface Disk surface (upper/lower)
 * @param decodedTrack Track structure to store decoded sector
 * @param ignoreHeaderChecksum Whether to ignore header checksum errors
 * @param lastSectorNumber Output: last sector number found
 * @return true if sector was decoded successfully
 */
inline bool decode_sector(const RawEncodedSector& rawSector, unsigned int trackNumber,
                          bool isHD, DiskSurface surface, DecodedTrack& decodedTrack,
                          bool ignoreHeaderChecksum, int& lastSectorNumber) {
    DecodedSector sector;
    lastSectorNumber = -1;
    std::memcpy(sector.rawSector, rawSector, sizeof(RawMFMData));
    
    const uint8_t* sectorData = rawSector;
    
    // Read first 4 bytes (track header)
    sector.headerChecksumCalculated = decode_mfm_data(
        reinterpret_cast<const uint32_t*>(sectorData + 8),
        reinterpret_cast<uint32_t*>(&sector), 4);
    
    // Decode label data and update checksum
    sector.headerChecksumCalculated ^= decode_mfm_data(
        reinterpret_cast<const uint32_t*>(sectorData + 16),
        sector.sectorLabel, 16);
    
    // Get header checksum
    decode_mfm_data(
        reinterpret_cast<const uint32_t*>(sectorData + 48),
        &sector.headerChecksum, 4);
    
    // Verify header checksum
    if ((sector.headerChecksum != sector.headerChecksumCalculated) && !ignoreHeaderChecksum) {
        return false;
    }
    
    // Validate header fields
    if (sector.trackFormat != 0xFF) return false;
    
    const size_t maxSectors = isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;
    if (sector.sectorNumber >= maxSectors) return false;
    if (sector.trackNumber > 166) return false;
    if (sector.sectorsRemaining > maxSectors) return false;
    if (sector.sectorsRemaining < 1) return false;
    
    // Verify track number matches expected
    const uint8_t targetTrackNumber = (trackNumber << 1) | (surface == DiskSurface::Upper ? 1 : 0);
    if (sector.trackNumber != targetTrackNumber) return false;
    
    // Get data checksum
    decode_mfm_data(
        reinterpret_cast<const uint32_t*>(sectorData + 56),
        &sector.dataChecksum, 4);
    
    // Check if we already have this sector as valid
    for (const auto& s : decodedTrack.validSectors) {
        if (s.sectorNumber == sector.sectorNumber) {
            return true;  // Already have it
        }
    }
    
    // Decode the data
    sector.dataChecksumCalculated = decode_mfm_data(
        reinterpret_cast<const uint32_t*>(sectorData + 64),
        reinterpret_cast<uint32_t*>(sector.data), SECTOR_BYTES);
    
    lastSectorNumber = sector.sectorNumber;
    
    // Verify data checksum
    if (sector.dataChecksum != sector.dataChecksumCalculated) {
        decodedTrack.invalidSectors[sector.sectorNumber].push_back(sector);
        return false;
    }
    
    // Valid sector
    decodedTrack.validSectors.push_back(sector);
    decodedTrack.invalidSectors[sector.sectorNumber].clear();
    
    return true;
}

/**
 * @brief Encode a sector into MFM format
 * 
 * @param trackNumber Physical track number
 * @param surface Disk surface
 * @param isHD Whether high-density
 * @param sectorNumber Sector number
 * @param input Raw sector data to encode
 * @param encodedSector Output encoded sector
 * @param lastByte Last byte from previous sector (for MFM clock bits)
 */
inline void encode_sector(unsigned int trackNumber, DiskSurface surface, bool isHD,
                          unsigned int sectorNumber, const RawDecodedSector& input,
                          RawEncodedSector& encodedSector, uint8_t& lastByte) {
    const size_t maxSectors = isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;
    
    // Sector start padding
    encodedSector[0] = (lastByte & 1) ? 0x2A : 0xAA;
    encodedSector[1] = 0xAA;
    encodedSector[2] = 0xAA;
    encodedSector[3] = 0xAA;
    
    // Sector sync words
    encodedSector[4] = 0x44;
    encodedSector[5] = 0x89;
    encodedSector[6] = 0x44;
    encodedSector[7] = 0x89;
    
    // Prepare header
    DecodedSector header;
    std::memset(&header, 0, sizeof(header));
    header.trackFormat = 0xFF;
    header.trackNumber = (trackNumber << 1) | (surface == DiskSurface::Upper ? 1 : 0);
    header.sectorNumber = sectorNumber;
    header.sectorsRemaining = maxSectors - sectorNumber;
    
    // Encode header
    header.headerChecksumCalculated = encode_mfm_data_part1(
        reinterpret_cast<const uint32_t*>(&header),
        reinterpret_cast<uint32_t*>(&encodedSector[8]), 4);
    
    // Encode label (zeros)
    header.headerChecksumCalculated ^= encode_mfm_data_part1(
        header.sectorLabel,
        reinterpret_cast<uint32_t*>(&encodedSector[16]), 16);
    
    // Encode header checksum
    encode_mfm_data_part1(
        &header.headerChecksumCalculated,
        reinterpret_cast<uint32_t*>(&encodedSector[48]), 4);
    
    // Encode data
    header.dataChecksumCalculated = encode_mfm_data_part1(
        reinterpret_cast<const uint32_t*>(input),
        reinterpret_cast<uint32_t*>(&encodedSector[64]), SECTOR_BYTES);
    
    // Encode data checksum
    encode_mfm_data_part1(
        &header.dataChecksumCalculated,
        reinterpret_cast<uint32_t*>(&encodedSector[56]), 4);
    
    // Fill in MFM clock bits
    bool lastBit = encodedSector[7] & 1;
    bool thisBit = lastBit;
    
    for (size_t count = 8; count < RAW_SECTOR_SIZE; count++) {
        for (int bit = 7; bit >= 1; bit -= 2) {
            lastBit = thisBit;
            thisBit = encodedSector[count] & (1 << (bit - 1));
            
            if (!(lastBit || thisBit)) {
                encodedSector[count] |= (1 << bit);
            }
        }
    }
    
    lastByte = encodedSector[RAW_SECTOR_SIZE - 1];
}

/**
 * @brief Find sectors in raw track data
 * 
 * Searches bit-by-bit for sync bytes and decodes sectors
 * 
 * @param track Raw track data
 * @param isHD Whether high-density disk
 * @param trackNumber Physical track number
 * @param side Disk surface
 * @param trackSync Sync word to search for
 * @param decodedTrack Output decoded track
 * @param ignoreHeaderChecksum Whether to ignore header checksum errors
 */
inline void find_sectors(const uint8_t* track, bool isHD, unsigned int trackNumber,
                         DiskSurface side, uint16_t trackSync, DecodedTrack& decodedTrack,
                         bool ignoreHeaderChecksum) {
    const uint32_t search = (trackSync | (static_cast<uint32_t>(trackSync) << 16));
    uint32_t decoded = 0;
    size_t byteIndex = 0;
    
    const size_t dataLength = isHD ? RAW_TRACKDATA_LENGTH_HD : RAW_TRACKDATA_LENGTH_DD;
    const size_t maxSectors = isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;
    
    while (byteIndex < dataLength) {
        for (int bitIndex = 7; bitIndex >= 0; bitIndex--) {
            decoded <<= 1;
            if (track[byteIndex] & (1 << bitIndex)) {
                decoded |= 1;
            }
            
            int lastSectorNumber = -1;
            if (decoded == search) {
                RawEncodedSector alignedSector;
                align_sector_to_byte(track, dataLength, byteIndex, bitIndex, alignedSector);
                
                if (decode_sector(alignedSector, trackNumber, isHD, side, decodedTrack,
                                  ignoreHeaderChecksum, lastSectorNumber)) {
                    byteIndex += RAW_SECTOR_SIZE - 8;
                    if (byteIndex >= dataLength) break;
                }
            }
        }
        byteIndex++;
    }
}

/**
 * @brief Merge invalid sectors into valid list (for error recovery)
 */
inline void merge_invalid_sectors(DecodedTrack& track, bool isHD) {
    const size_t maxSectors = isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;
    
    for (size_t sector = 0; sector < maxSectors; sector++) {
        if (!track.invalidSectors[sector].empty()) {
            DecodedSector sec = track.invalidSectors[sector][0];
            track.validSectors.push_back(sec);
        }
        track.invalidSectors[sector].clear();
    }
}

/**
 * @brief Convert flux data to raw MFM track data
 * 
 * Uses PLL-style decoding to convert flux transitions to MFM data
 * 
 * @param flux Flux data from disk
 * @param isHD Whether high-density disk
 * @param trackData Output raw track data
 * @return Number of bytes written to trackData
 */
inline size_t flux_to_mfm(const Flux& flux, bool isHD, uint8_t* trackData) {
    const size_t maxBytes = isHD ? RAW_TRACKDATA_LENGTH_HD : RAW_TRACKDATA_LENGTH_DD;
    
    // Target bit cell time in sample ticks
    // DD: 2us bit cells, HD: 1us bit cells
    const double bitCellTime = isHD ? 
        (flux.sample_freq / 1000000.0) :      // 1us for HD
        (flux.sample_freq / 500000.0);        // 2us for DD
    
    std::memset(trackData, 0, maxBytes);
    
    size_t byteIndex = 0;
    int bitIndex = 7;
    double accumulator = 0;
    
    // Skip to first index if available and flux is index-cued
    size_t fluxStart = 0;
    double ticksToSkip = 0;
    
    if (flux.index_cued && !flux.index_list.empty()) {
        // Already at index, use all flux
        ticksToSkip = 0;
    } else if (!flux.index_list.empty()) {
        // Need to skip to first index
        ticksToSkip = flux.index_list[0];
        double skipped = 0;
        for (size_t i = 0; i < flux.list.size() && skipped < ticksToSkip; i++) {
            skipped += flux.list[i];
            fluxStart = i + 1;
        }
    }
    
    // Target one revolution of data
    double targetTicks = 0;
    if (!flux.index_list.empty()) {
        targetTicks = flux.index_list[flux.index_cued ? 0 : 1];
    } else {
        // Estimate from flux data
        double totalFlux = 0;
        for (double f : flux.list) totalFlux += f;
        targetTicks = totalFlux;
    }
    
    double ticksProcessed = 0;
    
    for (size_t i = fluxStart; i < flux.list.size() && byteIndex < maxBytes; i++) {
        accumulator += flux.list[i];
        ticksProcessed += flux.list[i];
        
        // Stop after one revolution
        if (ticksProcessed > targetTicks * 1.1) break;
        
        // Count bit cells
        while (accumulator >= bitCellTime * 0.5) {
            if (accumulator >= bitCellTime * 1.5) {
                // Zero bit (long interval)
                accumulator -= bitCellTime;
                bitIndex--;
                if (bitIndex < 0) {
                    byteIndex++;
                    bitIndex = 7;
                }
            } else {
                // One bit (short interval at flux transition)
                trackData[byteIndex] |= (1 << bitIndex);
                accumulator -= bitCellTime;
                bitIndex--;
                if (bitIndex < 0) {
                    byteIndex++;
                    bitIndex = 7;
                }
                break;  // One flux = one '1' bit
            }
        }
    }
    
    return byteIndex + 1;
}

/**
 * @brief Convert MFM track data to flux transitions
 * 
 * @param trackData Raw MFM track data
 * @param numBytes Number of bytes in track data
 * @param isHD Whether high-density disk
 * @param sampleFreq Target sample frequency
 * @return Vector of flux timings
 */
inline std::vector<int> mfm_to_flux(const uint8_t* trackData, size_t numBytes,
                                     bool isHD, uint32_t sampleFreq) {
    std::vector<int> flux;
    
    // Bit cell time in sample ticks
    const double bitCellTime = isHD ?
        (sampleFreq / 1000000.0) :   // 1us for HD
        (sampleFreq / 500000.0);     // 2us for DD
    
    double accumulator = 0;
    
    for (size_t byteIndex = 0; byteIndex < numBytes; byteIndex++) {
        uint8_t byte = trackData[byteIndex];
        for (int bitIndex = 7; bitIndex >= 0; bitIndex--) {
            accumulator += bitCellTime;
            
            if (byte & (1 << bitIndex)) {
                // Flux transition
                flux.push_back(static_cast<int>(std::round(accumulator)));
                accumulator = 0;
            }
        }
    }
    
    // Add final accumulator if non-zero
    if (accumulator > 0) {
        flux.push_back(static_cast<int>(std::round(accumulator)));
    }
    
    return flux;
}

/**
 * @brief Callback function type for disk operations
 */
using ADFWriteCallback = std::function<WriteResponse(
    int currentTrack, DiskSurface currentSide, bool isVerifyError,
    CallbackOperation operation)>;

using ADFReadCallback = std::function<WriteResponse(
    int currentTrack, DiskSurface currentSide, int retryCounter,
    int sectorsFound, int badSectorsFound, int maxSectors,
    CallbackOperation operation)>;

/**
 * @brief Write an ADF file to a floppy disk
 * 
 * Reads sectors from an ADF file and writes them to the disk.
 * 
 * @param usb Greaseweazle device
 * @param inputFile Path to ADF file
 * @param isHD Whether writing high-density disk
 * @param verify Whether to verify after writing
 * @param callback Progress callback (optional)
 * @return Result of the operation
 */
inline ADFResult adf_to_disk(GreaseweazleUnit& usb, const std::string& inputFile,
                              bool isHD = false, bool verify = true,
                              ADFWriteCallback callback = nullptr) {
    // Notify starting
    if (callback) {
        if (callback(0, DiskSurface::Lower, false, CallbackOperation::Starting) == WriteResponse::Abort) {
            return ADFResult::Aborted;
        }
    }
    
    // Open ADF file
    std::ifstream adfFile(inputFile, std::ios::binary);
    if (!adfFile.is_open()) {
        return ADFResult::FileError;
    }
    
    // Get file size
    adfFile.seekg(0, std::ios::end);
    size_t fileSize = adfFile.tellg();
    adfFile.seekg(0, std::ios::beg);
    
    // Check for extended ADF header
    char header[8];
    adfFile.read(header, 8);
    if (std::strncmp(header, "UAE--ADF", 8) == 0) {
        return ADFResult::ExtendedADFNotSupported;
    }
    adfFile.seekg(0, std::ios::beg);
    
    // Determine disk type from file size
    const size_t DD_MAX_SIZE = ADF_TRACK_SIZE_DD * NUM_TRACKS_STANDARD * NUM_SIDES;
    const size_t HD_MAX_SIZE = ADF_TRACK_SIZE_HD * NUM_TRACKS_STANDARD * NUM_SIDES;
    
    bool fileIsHD = (fileSize > DD_MAX_SIZE);
    if (isHD != fileIsHD) {
        return ADFResult::MediaSizeMismatch;
    }
    
    const size_t adfTrackSize = isHD ? ADF_TRACK_SIZE_HD : ADF_TRACK_SIZE_DD;
    const size_t maxSectors = isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;
    
    // Allocate buffers
    std::vector<uint8_t> trackBuffer(adfTrackSize);
    FullDiskTrackDD diskTrackDD;
    FullDiskTrackHD diskTrackHD;
    
    std::memset(&diskTrackDD, 0xAA, sizeof(diskTrackDD));
    std::memset(&diskTrackHD, 0xAA, sizeof(diskTrackHD));
    
    // Setup drive
    usb.set_bus_type(BusType::Shugart);
    usb.drive_select(0);
    usb.drive_motor(0, true);
    
    // Wait for motor to spin up
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    bool errors = false;
    
    try {
        for (unsigned int currentTrack = 0; currentTrack < NUM_TRACKS_STANDARD; currentTrack++) {
            for (unsigned int surfaceIndex = 0; surfaceIndex < NUM_SIDES; surfaceIndex++) {
                DiskSurface surface = (surfaceIndex == 1) ? DiskSurface::Upper : DiskSurface::Lower;
                
                // Read track from file
                adfFile.read(reinterpret_cast<char*>(trackBuffer.data()), adfTrackSize);
                if (adfFile.gcount() != static_cast<std::streamsize>(adfTrackSize)) {
                    break;
                }
                
                if (callback) {
                    if (callback(currentTrack, surface, false, CallbackOperation::ReadingFile) == WriteResponse::Abort) {
                        usb.drive_motor(0, false);
                        usb.drive_deselect();
                        return ADFResult::Aborted;
                    }
                }
                
                // Seek to track
                usb.seek(currentTrack, surfaceIndex);
                
                // Encode sectors
                uint8_t lastByte = 0;
                if (isHD) {
                    lastByte = diskTrackHD.filler1[sizeof(diskTrackHD.filler1) - 1];
                    for (unsigned int sector = 0; sector < maxSectors; sector++) {
                        const uint8_t* sectorData = trackBuffer.data() + (sector * SECTOR_BYTES);
                        encode_sector(currentTrack, surface, isHD, sector,
                                     *reinterpret_cast<const RawDecodedSector*>(sectorData),
                                     diskTrackHD.sectors[sector], lastByte);
                    }
                    if (lastByte & 1) diskTrackHD.filler2[7] = 0x2F; else diskTrackHD.filler2[7] = 0xFF;
                } else {
                    lastByte = diskTrackDD.filler1[sizeof(diskTrackDD.filler1) - 1];
                    for (unsigned int sector = 0; sector < maxSectors; sector++) {
                        const uint8_t* sectorData = trackBuffer.data() + (sector * SECTOR_BYTES);
                        encode_sector(currentTrack, surface, isHD, sector,
                                     *reinterpret_cast<const RawDecodedSector*>(sectorData),
                                     diskTrackDD.sectors[sector], lastByte);
                    }
                    if (lastByte & 1) diskTrackDD.filler2[7] = 0x2F; else diskTrackDD.filler2[7] = 0xFF;
                }
                
                if (callback) {
                    if (callback(currentTrack, surface, false, CallbackOperation::Writing) == WriteResponse::Abort) {
                        usb.drive_motor(0, false);
                        usb.drive_deselect();
                        return ADFResult::Aborted;
                    }
                }
                
                // Convert to flux and write
                const uint8_t* dataPtr = isHD ? 
                    reinterpret_cast<const uint8_t*>(&diskTrackHD) :
                    reinterpret_cast<const uint8_t*>(&diskTrackDD);
                size_t dataSize = isHD ? sizeof(diskTrackHD) : sizeof(diskTrackDD);
                
                auto fluxList = mfm_to_flux(dataPtr, dataSize, isHD, usb.sample_freq);
                
                try {
                    usb.write_track(fluxList, true, true, 5);
                } catch (const CmdError& e) {
                    if (e.code == Ack::Wrprot) {
                        usb.drive_motor(0, false);
                        usb.drive_deselect();
                        return ADFResult::DiskWriteProtected;
                    }
                    throw;
                }
                
                // Verify if requested
                if (verify) {
                    if (callback) {
                        if (callback(currentTrack, surface, false, CallbackOperation::Verifying) == WriteResponse::Abort) {
                            usb.drive_motor(0, false);
                            usb.drive_deselect();
                            return ADFResult::Aborted;
                        }
                    }
                    
                    // Read back and verify
                    Flux readFlux = usb.read_track(2);
                    
                    // Convert flux to MFM data
                    RawTrackDataHD rawTrack;
                    std::memset(rawTrack, 0, sizeof(rawTrack));
                    flux_to_mfm(readFlux, isHD, rawTrack);
                    
                    // Decode sectors
                    DecodedTrack decodedTrack;
                    find_sectors(rawTrack, isHD, currentTrack, surface, AMIGA_WORD_SYNC, decodedTrack, false);
                    
                    if (decodedTrack.validSectors.size() < maxSectors) {
                        // Verification failed
                        if (callback) {
                            auto resp = callback(currentTrack, surface, true, CallbackOperation::ReVerifying);
                            if (resp == WriteResponse::Abort) {
                                usb.drive_motor(0, false);
                                usb.drive_deselect();
                                return ADFResult::Aborted;
                            } else if (resp == WriteResponse::SkipBadChecksums) {
                                errors = true;
                            }
                        } else {
                            errors = true;
                        }
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        usb.drive_motor(0, false);
        usb.drive_deselect();
        throw;
    }
    
    usb.drive_motor(0, false);
    usb.drive_deselect();
    adfFile.close();
    
    return errors ? ADFResult::CompletedWithErrors : ADFResult::Complete;
}

/**
 * @brief Read a floppy disk to an ADF file
 * 
 * Reads all tracks from the disk and saves them to an ADF file.
 * 
 * @param usb Greaseweazle device
 * @param outputFile Path to output ADF file
 * @param isHD Whether reading high-density disk
 * @param numTracks Number of tracks to read (default 80)
 * @param callback Progress callback (optional)
 * @return Result of the operation
 */
inline ADFResult disk_to_adf(GreaseweazleUnit& usb, const std::string& outputFile,
                              bool isHD = false, unsigned int numTracks = NUM_TRACKS_STANDARD,
                              ADFReadCallback callback = nullptr) {
    // Notify starting
    if (callback) {
        if (callback(0, DiskSurface::Lower, 0, 0, 0, 0, CallbackOperation::Starting) == WriteResponse::Abort) {
            return ADFResult::Aborted;
        }
    }
    
    // Limit tracks
    if (numTracks > 84) {
        numTracks = 84;
    }
    
    // Open output file
    std::ofstream adfFile(outputFile, std::ios::binary | std::ios::trunc);
    if (!adfFile.is_open()) {
        return ADFResult::FileError;
    }
    
    const size_t maxSectors = isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;
    
    // Setup drive
    usb.set_bus_type(BusType::Shugart);
    usb.drive_select(0);
    usb.drive_motor(0, true);
    
    // Wait for motor to spin up
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    bool includesBadSectors = false;
    RawTrackDataHD rawData;
    
    try {
        for (unsigned int currentTrack = 0; currentTrack < numTracks; currentTrack++) {
            for (unsigned int surfaceIndex = 0; surfaceIndex < NUM_SIDES; surfaceIndex++) {
                DiskSurface surface = (surfaceIndex == 1) ? DiskSurface::Upper : DiskSurface::Lower;
                
                // Seek to track
                usb.seek(currentTrack, surfaceIndex);
                
                // Reset decoded track
                DecodedTrack track;
                track.validSectors.clear();
                for (size_t i = 0; i < maxSectors; i++) {
                    track.invalidSectors[i].clear();
                }
                
                int failureTotal = 0;
                bool ignoreChecksums = false;
                
                // Read until we have all sectors
                while (track.validSectors.size() < maxSectors) {
                    if (callback) {
                        int badCount = 0;
                        for (size_t i = 0; i < maxSectors; i++) {
                            if (!track.invalidSectors[i].empty()) badCount++;
                        }
                        
                        auto resp = callback(currentTrack, surface, failureTotal,
                                           track.validSectors.size(), badCount, maxSectors,
                                           failureTotal > 0 ? CallbackOperation::RetryReading : CallbackOperation::Reading);
                        
                        switch (resp) {
                            case WriteResponse::Continue:
                                break;
                            case WriteResponse::Retry:
                                failureTotal = 0;
                                break;
                            case WriteResponse::Abort:
                                usb.drive_motor(0, false);
                                usb.drive_deselect();
                                adfFile.close();
                                return ADFResult::Aborted;
                            case WriteResponse::SkipBadChecksums:
                                if (ignoreChecksums) {
                                    // Create blank sectors for missing ones
                                    for (uint8_t sectorNum = 0; sectorNum < maxSectors; sectorNum++) {
                                        bool found = false;
                                        for (const auto& s : track.validSectors) {
                                            if (s.sectorNumber == sectorNum) {
                                                found = true;
                                                break;
                                            }
                                        }
                                        if (!found) {
                                            DecodedSector sector;
                                            std::memset(&sector, 0, sizeof(sector));
                                            sector.sectorNumber = sectorNum;
                                            track.validSectors.push_back(sector);
                                        }
                                    }
                                }
                                ignoreChecksums = true;
                                failureTotal = 0;
                                break;
                        }
                    }
                    
                    // Read track
                    Flux flux = usb.read_track(3);
                    
                    // Convert flux to MFM
                    std::memset(rawData, 0, sizeof(rawData));
                    flux_to_mfm(flux, isHD, rawData);
                    
                    // Find sectors
                    find_sectors(rawData, isHD, currentTrack, surface, AMIGA_WORD_SYNC,
                               track, ignoreChecksums);
                    
                    failureTotal++;
                    
                    // Limit retries
                    if (failureTotal > 20) {
                        if (ignoreChecksums) {
                            includesBadSectors = true;
                            merge_invalid_sectors(track, isHD);
                        } else if (callback) {
                            // Ask user what to do
                        } else {
                            includesBadSectors = true;
                            merge_invalid_sectors(track, isHD);
                        }
                        break;
                    }
                }
                
                // Sort sectors by sector number
                std::sort(track.validSectors.begin(), track.validSectors.end(),
                         [](const DecodedSector& a, const DecodedSector& b) {
                             return a.sectorNumber < b.sectorNumber;
                         });
                
                // Write sectors to file
                for (size_t sector = 0; sector < maxSectors && sector < track.validSectors.size(); sector++) {
                    adfFile.write(reinterpret_cast<const char*>(track.validSectors[sector].data),
                                  SECTOR_BYTES);
                }
                
                // Pad if missing sectors
                for (size_t sector = track.validSectors.size(); sector < maxSectors; sector++) {
                    std::array<uint8_t, SECTOR_BYTES> empty;
                    std::fill(empty.begin(), empty.end(), 0);
                    adfFile.write(reinterpret_cast<const char*>(empty.data()), SECTOR_BYTES);
                }
            }
        }
    } catch (const std::exception& e) {
        usb.drive_motor(0, false);
        usb.drive_deselect();
        adfFile.close();
        throw;
    }
    
    usb.drive_motor(0, false);
    usb.drive_deselect();
    adfFile.close();
    
    return includesBadSectors ? ADFResult::CompletedWithErrors : ADFResult::Complete;
}

/**
 * @brief Get result string for ADFResult
 */
inline const char* adf_result_string(ADFResult result) {
    switch (result) {
        case ADFResult::Complete: return "Complete";
        case ADFResult::CompletedWithErrors: return "Completed with errors";
        case ADFResult::Aborted: return "Aborted";
        case ADFResult::FileError: return "File error";
        case ADFResult::FileIOError: return "File I/O error";
        case ADFResult::DriveError: return "Drive error";
        case ADFResult::DiskWriteProtected: return "Disk is write protected";
        case ADFResult::MediaSizeMismatch: return "Media size mismatch";
        case ADFResult::ExtendedADFNotSupported: return "Extended ADF not supported";
        default: return "Unknown error";
    }
}

} // namespace amiga
} // namespace greaseweazle

#endif // GREASEWEAZLE_AMIGA_ADF_HPP
