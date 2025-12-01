/*
 * greaseweazle/flux.hpp
 *
 * Flux data structures and operations.
 *
 * C++ port using libusb 0.1
 * Original Python code written & released by Keir Fraser <keir.xen@gmail.com>
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#ifndef GREASEWEAZLE_FLUX_HPP
#define GREASEWEAZLE_FLUX_HPP

#include <vector>
#include <string>
#include <optional>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include "error.hpp"

namespace greaseweazle {

/**
 * @brief Forward declaration
 */
class WriteoutFlux;

/**
 * @brief Represents raw flux timing data from a floppy disk track
 */
class Flux {
public:
    std::vector<double> index_list;               ///< Index pulse timing list
    std::optional<std::vector<std::vector<double>>> sector_list; ///< Hard sector timing
    std::vector<double> list;                     ///< Flux timing samples
    double sample_freq;                           ///< Sample frequency in Hz
    std::optional<double> splice;                 ///< Splice position
    bool index_cued;                              ///< Whether flux is cued at index

private:
    double _ticks_per_rev = 0.0;

public:
    /**
     * @brief Construct a new Flux object
     * @param index_list List of index pulse timings
     * @param flux_list List of flux transition timings
     * @param sample_freq Sample frequency in Hz
     * @param index_cued Whether the flux is cued at index
     */
    Flux(const std::vector<double>& index_list,
         const std::vector<double>& flux_list,
         double sample_freq,
         bool index_cued = true)
        : index_list(index_list)
        , list(flux_list)
        , sample_freq(sample_freq)
        , index_cued(index_cued)
    {}

    /**
     * @brief Convert to string representation
     */
    std::string to_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "\nFlux: " << (sample_freq * 1e-6) << " MHz";
        if (index_cued) ss << ", Index-Cued";
        
        double total_flux = std::accumulate(list.begin(), list.end(), 0.0);
        ss << "\n Total: " << list.size() << " samples, " 
           << (total_flux * 1000 / sample_freq) << "ms\n";
        
        for (size_t rev = 0; rev < index_list.size(); ++rev) {
            ss << " Revolution " << rev << ": " 
               << (index_list[rev] * 1000 / sample_freq) << "ms\n";
            if (sector_list && rev < sector_list->size()) {
                for (size_t sec = 0; sec < (*sector_list)[rev].size(); ++sec) {
                    ss << "    Sector " << sec << ": "
                       << ((*sector_list)[rev][sec] * 1000 / sample_freq) << "ms\n";
                }
            }
        }
        return ss.str();
    }

    /**
     * @brief Get a summary string
     */
    std::string summary_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        double total_flux = std::accumulate(list.begin(), list.end(), 0.0);
        ss << "Raw Flux (" << list.size() << " flux in "
           << (total_flux * 1000 / sample_freq) << "ms)";
        return ss.str();
    }

    /**
     * @brief Get mean time between index pulses in sample ticks
     */
    double ticks_per_rev() const {
        try {
            std::vector<double> idx = index_list;
            if (!index_cued && !idx.empty()) {
                idx.erase(idx.begin());
            }
            if (idx.empty()) {
                return _ticks_per_rev;
            }
            double sum = std::accumulate(idx.begin(), idx.end(), 0.0);
            return sum / idx.size();
        } catch (...) {
            return _ticks_per_rev;
        }
    }

    /**
     * @brief Get mean time between index pulses in seconds
     */
    double time_per_rev() const {
        return ticks_per_rev() / sample_freq;
    }

    /**
     * @brief Cue flux data at index
     */
    void cue_at_index() {
        if (index_cued) return;
        
        check(index_list.size() >= 2,
              "Not enough revolutions of flux data to cue at index. "
              "Try dumping more revolutions (larger --revs value).");

        // Clip the initial partial revolution
        double to_index = index_list[0];
        size_t i = 0;
        for (i = 0; i < list.size(); ++i) {
            to_index -= list[i];
            if (to_index < 0) break;
        }
        
        if (to_index < 0) {
            std::vector<double> new_list;
            new_list.push_back(-to_index);
            new_list.insert(new_list.end(), list.begin() + i + 1, list.end());
            list = std::move(new_list);
        } else {
            list.clear();
        }
        
        index_list.erase(index_list.begin());
        index_cued = true;
        
        if (sector_list && !sector_list->empty()) {
            sector_list->erase(sector_list->begin());
        }
    }

    /**
     * @brief Reverse flux data
     */
    void reverse() {
        bool was_index_cued = index_cued;
        double flux_sum = std::accumulate(list.begin(), list.end(), 0.0);

        index_cued = false;
        std::reverse(list.begin(), list.end());
        std::reverse(index_list.begin(), index_list.end());

        double index_sum = std::accumulate(index_list.begin(), index_list.end(), 0.0);
        double to_index = flux_sum - index_sum;
        
        if (to_index <= 0) {
            if (to_index < 0) {
                list.insert(list.begin(), -to_index);
                flux_sum += -to_index;
            }
            if (!index_list.empty()) {
                index_list.erase(index_list.begin());
            }
            index_cued = true;
        } else {
            if (!index_list.empty()) {
                index_list.pop_back();
            }
            index_list.insert(index_list.begin(), to_index);
        }

        if (was_index_cued) {
            double new_index_sum = std::accumulate(index_list.begin(), index_list.end(), 0.0);
            index_list.push_back(flux_sum - new_index_sum);
        }
    }

    /**
     * @brief Append another flux object
     */
    void append(const Flux& flux) {
        std::vector<double> f_list, i_list;
        
        if (sample_freq == flux.sample_freq) {
            f_list = flux.list;
            i_list = flux.index_list;
        } else {
            double factor = sample_freq / flux.sample_freq;
            f_list.reserve(flux.list.size());
            for (double x : flux.list) {
                f_list.push_back(x * factor);
            }
            i_list.reserve(flux.index_list.size());
            for (double x : flux.index_list) {
                i_list.push_back(x * factor);
            }
        }

        double list_sum = std::accumulate(list.begin(), list.end(), 0.0);
        double index_sum = std::accumulate(index_list.begin(), index_list.end(), 0.0);
        double rev0 = i_list[0] + list_sum - index_sum;
        
        index_list.push_back(rev0);
        for (size_t i = 1; i < i_list.size(); ++i) {
            index_list.push_back(i_list[i]);
        }
        list.insert(list.end(), f_list.begin(), f_list.end());
        
        sector_list.reset();
    }

    /**
     * @brief Scale flux data by factor
     */
    void scale(double factor) {
        sample_freq /= factor;
    }

    /**
     * @brief Get flux for writeout
     */
    WriteoutFlux flux_for_writeout(bool cue_at_index) const;

    /**
     * @brief Get self reference (for protocol compatibility)
     */
    Flux& flux() { return *this; }
    const Flux& flux() const { return *this; }

    /**
     * @brief Set internal ticks per revolution
     */
    void set_ticks_per_rev(double tpr) { _ticks_per_rev = tpr; }
};

/**
 * @brief Represents flux data prepared for writing to disk
 */
class WriteoutFlux {
public:
    double ticks_to_index;                ///< Ticks until next index
    std::vector<double> list;             ///< Flux timing samples
    double sample_freq;                   ///< Sample frequency in Hz
    bool index_cued;                      ///< Whether cued at index
    bool terminate_at_index;              ///< Whether to terminate at index

    WriteoutFlux(double ticks_to_index,
                 const std::vector<double>& flux_list,
                 double sample_freq,
                 bool index_cued,
                 bool terminate_at_index)
        : ticks_to_index(ticks_to_index)
        , list(flux_list)
        , sample_freq(sample_freq)
        , index_cued(index_cued)
        , terminate_at_index(terminate_at_index)
    {}

    std::string to_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        double total_flux = std::accumulate(list.begin(), list.end(), 0.0);
        ss << "\nWriteoutFlux: " << (sample_freq * 1e-6) << " MHz, "
           << (ticks_to_index * 1000 / sample_freq) << "ms to index, "
           << (terminate_at_index ? "Terminate at index" : "Write all")
           << "\n Total: " << list.size() << " samples, "
           << (total_flux * 1000 / sample_freq) << "ms";
        return ss.str();
    }

    std::string summary_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1);
        double total_flux = std::accumulate(list.begin(), list.end(), 0.0);
        ss << "Flux: " << (ticks_to_index * 1000 / sample_freq) << "ms period, "
           << (total_flux * 1000 / sample_freq) << " ms total, "
           << (terminate_at_index ? "Terminate at index" : "Write all");
        return ss.str();
    }
};

// Implementation of flux_for_writeout
inline WriteoutFlux Flux::flux_for_writeout(bool cue_at_index_param) const {
    double splice_val = splice.value_or(0);

    check(index_cued, "Cannot write non-index-cued raw flux");
    check(splice_val == 0 || index_list.size() > 1,
          "Cannot write single-revolution unaligned raw flux");
    
    bool splice_at_index = (splice_val == 0);

    std::vector<double> flux_list;
    double to_index = index_list[0];
    double remain = to_index + splice_val;
    
    for (double f : list) {
        if (f > remain) break;
        flux_list.push_back(f);
        remain -= f;
    }

    if (!cue_at_index_param) {
        if (remain > 0) {
            flux_list.push_back(remain);
        }
        double prepend = std::max(static_cast<double>(std::round(to_index / 10 - splice_val)), 0.0);
        if (prepend != 0) {
            double four_us = std::max(sample_freq * 4e-6, 1.0);
            std::vector<double> prepend_flux(static_cast<size_t>(std::round(prepend / four_us)), four_us);
            prepend_flux.insert(prepend_flux.end(), flux_list.begin(), flux_list.end());
            flux_list = std::move(prepend_flux);
        }
        splice_at_index = false;
    } else if (splice_at_index) {
        double four_us = std::max(sample_freq * 4e-6, 1.0);
        if (remain > four_us) {
            flux_list.push_back(remain);
        }
        for (size_t i = 0; i < static_cast<size_t>(std::round(to_index / (10 * four_us))); ++i) {
            flux_list.push_back(four_us);
        }
    } else if (remain > 0) {
        flux_list.push_back(remain);
    }

    return WriteoutFlux(to_index, flux_list, sample_freq,
                        cue_at_index_param, splice_at_index);
}

} // namespace greaseweazle

#endif // GREASEWEAZLE_FLUX_HPP
