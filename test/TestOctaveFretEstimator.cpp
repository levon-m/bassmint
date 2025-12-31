/**
 * Test harness for OctaveFretEstimator
 *
 * Generates synthetic bass signals to validate octave disambiguation.
 * Compile with:
 *   g++ -std=c++17 -O2 -I../src -o test_octave \
 *       TestOctaveFretEstimator.cpp \
 *       ../src/dsp/OctaveFretEstimator.cpp \
 *       ../src/core/Tuning.cpp
 *
 * Or use the provided CMakeLists.txt
 */

#include "../src/dsp/OctaveFretEstimator.h"
#include "../src/core/Tuning.h"
#include <cmath>
#include <cstdio>
#include <vector>
#include <random>

namespace bassmint {

// =============================================================================
// Synthetic Signal Generator
// =============================================================================

class SyntheticBassSignal {
public:
    /**
     * Generate a bass signal with harmonics
     * @param fundamentalHz Fundamental frequency
     * @param sampleRate Sample rate
     * @param durationMs Duration in milliseconds
     * @param harmonicStrengths Relative strengths of harmonics [f, 2f, 3f, ...]
     * @param noiseLevel RMS noise level (0-1)
     * @param amplitudeEnvelope If true, apply attack/sustain/release envelope
     */
    static std::vector<float> generate(
        float fundamentalHz,
        float sampleRate,
        float durationMs,
        const std::vector<float>& harmonicStrengths = {1.0f, 0.5f, 0.3f, 0.2f, 0.1f},
        float noiseLevel = 0.02f,
        bool amplitudeEnvelope = true
    ) {
        size_t numSamples = static_cast<size_t>(sampleRate * durationMs / 1000.0f);
        std::vector<float> signal(numSamples);

        std::mt19937 rng(42);  // Fixed seed for reproducibility
        std::normal_distribution<float> noise(0.0f, noiseLevel);

        float attackMs = 10.0f;
        float releaseMs = 50.0f;
        size_t attackSamples = static_cast<size_t>(attackMs * sampleRate / 1000.0f);
        size_t releaseSamples = static_cast<size_t>(releaseMs * sampleRate / 1000.0f);

        for (size_t i = 0; i < numSamples; ++i) {
            float t = static_cast<float>(i) / sampleRate;

            // Generate harmonics
            float sample = 0.0f;
            for (size_t h = 0; h < harmonicStrengths.size(); ++h) {
                float freq = fundamentalHz * (h + 1);
                sample += harmonicStrengths[h] * std::sin(2.0f * 3.14159265f * freq * t);
            }

            // Add noise
            sample += noise(rng);

            // Apply envelope
            if (amplitudeEnvelope) {
                float env = 1.0f;
                if (i < attackSamples) {
                    env = static_cast<float>(i) / attackSamples;
                } else if (i > numSamples - releaseSamples) {
                    env = static_cast<float>(numSamples - i) / releaseSamples;
                }
                sample *= env;
            }

            // Simulate OPT101 DC offset (centered around 0.5)
            signal[i] = 0.5f + 0.3f * sample;
        }

        return signal;
    }

    /**
     * Generate signal with strong 2nd harmonic (octave confusion test)
     */
    static std::vector<float> generateStrongSecondHarmonic(
        float fundamentalHz,
        float sampleRate,
        float durationMs
    ) {
        // 2nd harmonic stronger than fundamental!
        return generate(fundamentalHz, sampleRate, durationMs,
                       {0.4f, 1.0f, 0.3f, 0.2f, 0.1f});
    }
};

// =============================================================================
// Test Cases
// =============================================================================

void printEstimate(const FretEstimate& est, const OctaveDebugInfo& dbg) {
    printf("  Fret: %2d  Conf: %.3f  Freq: %.1f Hz  Octave: %s\n",
           est.fret, est.confidence, est.frequencyHz,
           est.octaveState == 0 ? "LOW" : (est.octaveState == 1 ? "HIGH" : "?"));
    printf("  Debug: peakLow=%.3f peakHigh=%.3f beliefLow=%.3f beliefHigh=%.3f\n",
           dbg.peakLow, dbg.peakHigh, dbg.beliefLow, dbg.beliefHigh);
    printf("         freqLow=%.1f freqHigh=%.1f rawFret=%d\n",
           dbg.freqLow, dbg.freqHigh, dbg.rawFret);
}

/**
 * Test 1: Basic pitch detection on each string
 */
bool testBasicPitchDetection() {
    printf("\n=== Test 1: Basic Pitch Detection ===\n");

    OctaveFretEstimator estimator;
    bool allPassed = true;

    struct TestCase {
        int stringIdx;
        int fret;
        const char* name;
    };

    TestCase cases[] = {
        {0, 0, "E string open (E1, 41.2 Hz)"},
        {0, 5, "E string fret 5 (A1, 55 Hz)"},
        {1, 0, "A string open (A1, 55 Hz)"},
        {1, 7, "A string fret 7 (E2, 82.4 Hz)"},
        {2, 0, "D string open (D2, 73.4 Hz)"},
        {2, 5, "D string fret 5 (G2, 98 Hz)"},
        {3, 0, "G string open (G2, 98 Hz)"},
        {3, 12, "G string fret 12 (G3, 196 Hz)"},
    };

    for (const auto& tc : cases) {
        printf("\nTesting: %s\n", tc.name);

        estimator.reset();

        float openFreq = Tuning::OpenStringHz[tc.stringIdx];
        float targetFreq = openFreq * std::pow(2.0f, tc.fret / 12.0f);

        auto signal = SyntheticBassSignal::generate(
            targetFreq,
            OctaveConfig::SourceSampleRate,
            200.0f  // 200ms signal
        );

        estimator.onStringActive(tc.stringIdx);

        // Process all samples
        FretEstimate lastEstimate;
        OctaveDebugInfo lastDebug;
        int frameCount = 0;

        for (float sample : signal) {
            estimator.processSample(sample);
            if (estimator.hasNewEstimate()) {
                lastEstimate = estimator.getEstimate();
                lastDebug = estimator.getDebugInfo();
                ++frameCount;
            }
        }

        printf("Processed %d frames\n", frameCount);
        printEstimate(lastEstimate, lastDebug);

        bool passed = (lastEstimate.fret == tc.fret);
        printf("Expected fret %d, got %d: %s\n",
               tc.fret, lastEstimate.fret,
               passed ? "PASS" : "FAIL");

        if (!passed) allPassed = false;
    }

    return allPassed;
}

/**
 * Test 2: Octave disambiguation - the critical test
 * A-string fret 3 (C2, 65.4 Hz) vs fret 15 (C3, 130.8 Hz)
 */
bool testOctaveDisambiguation() {
    printf("\n=== Test 2: Octave Disambiguation ===\n");

    OctaveFretEstimator estimator;
    bool allPassed = true;

    // Test A-string: fret 3 vs fret 15 (both are C, one octave apart)
    int stringIdx = 1;  // A string
    float openFreq = Tuning::OpenStringHz[stringIdx];

    struct OctaveTest {
        int fret;
        float frequency;
        const char* name;
    };

    OctaveTest tests[] = {
        {3, openFreq * std::pow(2.0f, 3.0f / 12.0f), "A string fret 3 (C2, LOW octave)"},
        {15, openFreq * std::pow(2.0f, 15.0f / 12.0f), "A string fret 15 (C3, HIGH octave)"},
    };

    for (const auto& test : tests) {
        printf("\nTesting: %s (%.1f Hz)\n", test.name, test.frequency);

        estimator.reset();

        // Normal harmonic content
        auto signal = SyntheticBassSignal::generate(
            test.frequency,
            OctaveConfig::SourceSampleRate,
            250.0f  // 250ms signal
        );

        estimator.onStringActive(stringIdx);

        FretEstimate lastEstimate;
        OctaveDebugInfo lastDebug;

        for (float sample : signal) {
            estimator.processSample(sample);
            if (estimator.hasNewEstimate()) {
                lastEstimate = estimator.getEstimate();
                lastDebug = estimator.getDebugInfo();
            }
        }

        printEstimate(lastEstimate, lastDebug);

        bool passed = (lastEstimate.fret == test.fret);
        printf("Expected fret %d, got %d: %s\n",
               test.fret, lastEstimate.fret,
               passed ? "PASS" : "FAIL");

        if (!passed) allPassed = false;
    }

    return allPassed;
}

/**
 * Test 3: Strong 2nd harmonic (common problem case)
 * The 2nd harmonic is an octave up, which can confuse pitch detectors
 */
bool testStrongSecondHarmonic() {
    printf("\n=== Test 3: Strong 2nd Harmonic ===\n");

    OctaveFretEstimator estimator;
    bool allPassed = true;

    // A string fret 3 (C2, 65.4 Hz) with strong 2nd harmonic at 130.8 Hz
    int stringIdx = 1;
    int expectedFret = 3;
    float openFreq = Tuning::OpenStringHz[stringIdx];
    float targetFreq = openFreq * std::pow(2.0f, expectedFret / 12.0f);

    printf("\nTesting: A string fret 3 with strong 2nd harmonic\n");
    printf("Fundamental: %.1f Hz, 2nd harmonic: %.1f Hz\n", targetFreq, targetFreq * 2);

    estimator.reset();

    auto signal = SyntheticBassSignal::generateStrongSecondHarmonic(
        targetFreq,
        OctaveConfig::SourceSampleRate,
        300.0f
    );

    estimator.onStringActive(stringIdx);

    FretEstimate lastEstimate;
    OctaveDebugInfo lastDebug;
    int frameCount = 0;

    for (float sample : signal) {
        estimator.processSample(sample);
        if (estimator.hasNewEstimate()) {
            lastEstimate = estimator.getEstimate();
            lastDebug = estimator.getDebugInfo();
            ++frameCount;

            // Print each frame for debugging
            if (frameCount <= 5 || frameCount % 5 == 0) {
                printf("Frame %d: ", frameCount);
                printf("fret=%d belief=[%.2f,%.2f] peaks=[%.2f,%.2f]\n",
                       lastEstimate.fret, lastDebug.beliefLow, lastDebug.beliefHigh,
                       lastDebug.peakLow, lastDebug.peakHigh);
            }
        }
    }

    printf("\nFinal result:\n");
    printEstimate(lastEstimate, lastDebug);

    // With strong 2nd harmonic, we might initially get confused
    // but the Bayesian filter should eventually settle on correct octave
    bool passed = (lastEstimate.fret == expectedFret);
    printf("Expected fret %d, got %d: %s\n",
           expectedFret, lastEstimate.fret,
           passed ? "PASS" : "FAIL");

    if (!passed) {
        printf("Note: Octave error detected (fret difference of 12)\n");
        if (std::abs(lastEstimate.fret - expectedFret) == 12) {
            printf("This is the classic octave confusion case we're trying to solve!\n");
        }
    }

    return passed;
}

/**
 * Test 4: Sequence of notes with hand-span prior
 */
bool testHandSpanPrior() {
    printf("\n=== Test 4: Hand-Span Prior (Sequential Notes) ===\n");

    OctaveFretEstimator estimator;
    bool allPassed = true;

    // Play a sequence: fret 3 -> fret 5 -> fret 7 -> fret 3
    // The hand-span prior should keep us in the low octave
    int stringIdx = 1;  // A string
    float openFreq = Tuning::OpenStringHz[stringIdx];

    int sequence[] = {3, 5, 7, 3};

    estimator.reset();
    estimator.onStringActive(stringIdx);

    for (int targetFret : sequence) {
        float targetFreq = openFreq * std::pow(2.0f, targetFret / 12.0f);

        printf("\nPlaying fret %d (%.1f Hz):\n", targetFret, targetFreq);

        auto signal = SyntheticBassSignal::generate(
            targetFreq,
            OctaveConfig::SourceSampleRate,
            150.0f
        );

        FretEstimate lastEstimate;
        OctaveDebugInfo lastDebug;

        for (float sample : signal) {
            estimator.processSample(sample);
            if (estimator.hasNewEstimate()) {
                lastEstimate = estimator.getEstimate();
                lastDebug = estimator.getDebugInfo();
            }
        }

        printEstimate(lastEstimate, lastDebug);

        bool passed = (lastEstimate.fret == targetFret);
        printf("Expected fret %d, got %d: %s\n",
               targetFret, lastEstimate.fret,
               passed ? "PASS" : "FAIL");

        if (!passed) allPassed = false;
    }

    return allPassed;
}

/**
 * Test 5: All strings, all octave boundaries
 */
bool testAllOctaveBoundaries() {
    printf("\n=== Test 5: Octave Boundaries on All Strings ===\n");

    OctaveFretEstimator estimator;
    bool allPassed = true;

    // Test frets 11 and 12 (the octave boundary) on each string
    for (int stringIdx = 0; stringIdx < 4; ++stringIdx) {
        float openFreq = Tuning::OpenStringHz[stringIdx];
        const char* stringName = Tuning::StringNames[stringIdx];

        for (int fret : {11, 12}) {
            float targetFreq = openFreq * std::pow(2.0f, fret / 12.0f);

            printf("\n%s string fret %d (%.1f Hz):\n", stringName, fret, targetFreq);

            estimator.reset();
            estimator.onStringActive(stringIdx);

            auto signal = SyntheticBassSignal::generate(
                targetFreq,
                OctaveConfig::SourceSampleRate,
                200.0f
            );

            FretEstimate lastEstimate;
            OctaveDebugInfo lastDebug;

            for (float sample : signal) {
                estimator.processSample(sample);
                if (estimator.hasNewEstimate()) {
                    lastEstimate = estimator.getEstimate();
                    lastDebug = estimator.getDebugInfo();
                }
            }

            bool passed = (lastEstimate.fret == fret);
            printf("  Got fret %d (expected %d): %s\n",
                   lastEstimate.fret, fret,
                   passed ? "PASS" : "FAIL");

            if (!passed) allPassed = false;
        }
    }

    return allPassed;
}

} // namespace bassmint

// =============================================================================
// Main
// =============================================================================

int main() {
    printf("OctaveFretEstimator Test Harness\n");
    printf("================================\n\n");

    printf("Configuration:\n");
    printf("  Source sample rate: %.0f Hz\n", bassmint::OctaveConfig::SourceSampleRate);
    printf("  Decimated rate: %.0f Hz\n", bassmint::OctaveConfig::DecimatedSampleRate);
    printf("  Window size (E): %zu samples (%.1f ms)\n",
           bassmint::OctaveConfig::WindowSizeE,
           1000.0f * bassmint::OctaveConfig::WindowSizeE / bassmint::OctaveConfig::DecimatedSampleRate);
    printf("  Window size (other): %zu samples (%.1f ms)\n",
           bassmint::OctaveConfig::WindowSizeDefault,
           1000.0f * bassmint::OctaveConfig::WindowSizeDefault / bassmint::OctaveConfig::DecimatedSampleRate);
    printf("  Hop size: %zu samples (%.1f ms)\n",
           bassmint::OctaveConfig::HopSize,
           1000.0f * bassmint::OctaveConfig::HopSize / bassmint::OctaveConfig::DecimatedSampleRate);

    int passed = 0;
    int failed = 0;

    if (bassmint::testBasicPitchDetection()) ++passed; else ++failed;
    if (bassmint::testOctaveDisambiguation()) ++passed; else ++failed;
    if (bassmint::testStrongSecondHarmonic()) ++passed; else ++failed;
    if (bassmint::testHandSpanPrior()) ++passed; else ++failed;
    if (bassmint::testAllOctaveBoundaries()) ++passed; else ++failed;

    printf("\n================================\n");
    printf("Results: %d passed, %d failed\n", passed, failed);
    printf("================================\n");

    return failed > 0 ? 1 : 0;
}
