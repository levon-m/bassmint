#pragma once

#include <cstddef>
#include <array>
#include <algorithm>

namespace bassmint {

/**
 * Fixed-size ring buffer for audio sample processing
 *
 * Features:
 * - Zero-copy read access via data pointer
 * - Efficient push with automatic wrap-around
 * - Support for bulk operations
 * - Contiguous data access for DSP algorithms
 *
 * @tparam T Sample type (typically float)
 * @tparam N Maximum buffer size
 */
template <typename T, size_t N>
class RingBuffer {
public:
    /**
     * Construct empty ring buffer
     */
    RingBuffer() : writeIndex_(0), count_(0) {
        buffer_.fill(T{});
    }

    /**
     * Push a single sample into the buffer
     * @param sample Sample to push
     */
    void push(T sample) {
        buffer_[writeIndex_] = sample;
        writeIndex_ = (writeIndex_ + 1) % N;
        if (count_ < N) {
            ++count_;
        }
    }

    /**
     * Push multiple samples into the buffer
     * @param samples Pointer to samples
     * @param numSamples Number of samples to push
     */
    void push(const T* samples, size_t numSamples) {
        for (size_t i = 0; i < numSamples; ++i) {
            push(samples[i]);
        }
    }

    /**
     * Get sample at index (0 = oldest, count-1 = newest)
     */
    T operator[](size_t index) const {
        if (index >= count_) return T{};
        size_t actualIndex = (writeIndex_ + N - count_ + index) % N;
        return buffer_[actualIndex];
    }

    /**
     * Get the most recent sample
     */
    T newest() const {
        if (count_ == 0) return T{};
        return buffer_[(writeIndex_ + N - 1) % N];
    }

    /**
     * Get the oldest sample
     */
    T oldest() const {
        if (count_ == 0) return T{};
        return buffer_[(writeIndex_ + N - count_) % N];
    }

    /**
     * Copy the last 'numSamples' samples to a contiguous output buffer
     * Samples are ordered oldest to newest
     *
     * @param output Destination buffer (must have space for numSamples)
     * @param numSamples Number of samples to copy
     * @return Actual number of samples copied
     */
    size_t copyTo(T* output, size_t numSamples) const {
        size_t toCopy = std::min(numSamples, count_);
        for (size_t i = 0; i < toCopy; ++i) {
            output[i] = (*this)[count_ - toCopy + i];
        }
        return toCopy;
    }

    /**
     * Copy the most recent samples to a contiguous buffer
     * @param output Destination buffer
     * @param numSamples Number of samples to copy
     * @return Actual number copied
     */
    size_t copyNewest(T* output, size_t numSamples) const {
        size_t toCopy = std::min(numSamples, count_);
        size_t startIdx = count_ - toCopy;
        for (size_t i = 0; i < toCopy; ++i) {
            output[i] = (*this)[startIdx + i];
        }
        return toCopy;
    }

    /**
     * Get current number of samples in buffer
     */
    size_t size() const { return count_; }

    /**
     * Get maximum capacity
     */
    constexpr size_t capacity() const { return N; }

    /**
     * Check if buffer is full
     */
    bool isFull() const { return count_ == N; }

    /**
     * Check if buffer is empty
     */
    bool isEmpty() const { return count_ == 0; }

    /**
     * Clear all samples
     */
    void clear() {
        writeIndex_ = 0;
        count_ = 0;
        buffer_.fill(T{});
    }

    /**
     * Reset write position but keep data (for re-reading)
     */
    void resetWrite() {
        writeIndex_ = 0;
        count_ = 0;
    }

    /**
     * Get direct read-only access to internal buffer
     * Note: Data may not be contiguous - use copyTo() for contiguous access
     */
    const std::array<T, N>& rawBuffer() const { return buffer_; }

    /**
     * Get write index (for advanced usage)
     */
    size_t writeIndex() const { return writeIndex_; }

private:
    std::array<T, N> buffer_;
    size_t writeIndex_;
    size_t count_;
};

} // namespace bassmint
