// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#ifndef EASYFLY_FASTLIO_SCAN_QUEUE_HPP_
#define EASYFLY_FASTLIO_SCAN_QUEUE_HPP_
#include <cstddef>
#include <deque>
#include <utility>

namespace easyfly_fastlio {
// Keep acquisition time inseparable from its cloud. Under processing pressure
// discard the oldest unprocessed scan; IMU history is owned separately and is
// still integrated from the last processed scan through the next scan's end.
template<class Cloud> class ScanQueue {
public:
    struct Scan { Cloud cloud; double stamp; };
    static constexpr std::size_t capacity = 2;
    bool push(Cloud cloud, double stamp) {
        const bool replaced_front = scans_.size() == capacity;
        if (replaced_front) { scans_.pop_front(); ++dropped_; }
        scans_.push_back({std::move(cloud), stamp});
        return replaced_front;
    }
    bool empty() const { return scans_.empty(); }
    std::size_t size() const { return scans_.size(); }
    std::size_t dropped() const { return dropped_; }
    const Scan& front() const { return scans_.front(); }
    void pop_front() { scans_.pop_front(); }
    void clear() { scans_.clear(); }
private:
    std::deque<Scan> scans_;
    std::size_t dropped_{0};
};
}  // namespace easyfly_fastlio
#endif
