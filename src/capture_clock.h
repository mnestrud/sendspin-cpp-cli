// Copyright 2026 sendspin-cpp-cli Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// Capture timestamps from the sample count, steered slowly toward the device's own stamps.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace sendspin_cli {

/// A server lines source chunks up against its first stamp plus the frames it has been fed and
/// inserts silence for a chunk stamped more than about 15 ms off that line. Per-read device stamps
/// carry wake-up and clock-read jitter of that order, so each chunk is stamped from the frames
/// captured since an anchor, and a low-pass correction follows the device stamps for clock drift.
/// A device stamp further than REANCHOR_US off the line (an overrun, a reopen, a pause between
/// streams) starts a new anchor.
class CaptureClock {
public:
    static constexpr int64_t REANCHOR_US = 50000;
    /// Fraction of each read's error taken into the correction (about a 1.3 s time constant at
    /// 20 ms reads).
    static constexpr double ALPHA = 1.0 / 64.0;

    explicit CaptureClock(uint32_t sample_rate) : rate_(sample_rate == 0 ? 1 : sample_rate) {}

    void reset() {
        this->anchored_ = false;
    }

    /// @param measured The device's stamp for the first frame of this read, 0 if unknown.
    /// @param frames Frames in this read.
    /// @return The stamp to send, 0 while nothing is known (the role then stamps it itself).
    int64_t stamp(int64_t measured, size_t frames) {
        int64_t out = 0;
        if (this->anchored_) {
            out = this->anchor_ +
                  static_cast<int64_t>(this->frames_ * 1000000ULL / this->rate_) +
                  static_cast<int64_t>(this->correction_);
        }
        if (measured != 0) {
            const int64_t error = this->anchored_ ? measured - out : 0;
            if (!this->anchored_ || std::llabs(error) > REANCHOR_US) {
                if (this->anchored_) {
                    ++this->reanchors_;
                }
                this->anchored_ = true;
                this->anchor_ = measured;
                this->frames_ = 0;
                this->correction_ = 0.0;
                out = measured;
            } else {
                this->correction_ += static_cast<double>(error) * ALPHA;
                const int64_t magnitude = std::llabs(error);
                if (magnitude > this->max_error_us_) {
                    this->max_error_us_ = magnitude;
                }
            }
        }
        if (this->anchored_) {
            this->frames_ += frames;
        }
        return out;
    }

    /// Largest |device stamp - sent stamp| since the last call, and resets it.
    int64_t take_max_error_us() {
        const int64_t m = this->max_error_us_;
        this->max_error_us_ = 0;
        return m;
    }
    uint32_t reanchors() const {
        return this->reanchors_;
    }
    int64_t correction_us() const {
        return static_cast<int64_t>(this->correction_);
    }

private:
    uint32_t rate_;
    bool anchored_{false};
    int64_t anchor_{0};
    uint64_t frames_{0};
    double correction_{0.0};
    int64_t max_error_us_{0};
    uint32_t reanchors_{0};
};

}  // namespace sendspin_cli
