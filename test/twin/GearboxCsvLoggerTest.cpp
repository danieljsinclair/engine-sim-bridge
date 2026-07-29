// GearboxCsvLoggerTest.cpp - Contract tests for GearboxCsvLogger
//
// GearboxCsvLogger.cpp is a dead-stripped file (40 LOC, 0% coverage) that writes
// a CSV log of gearbox telemetry to a file. These tests assert the OBSERVABLE
// contract by reading the produced file back:
//   - writes a header exactly once, then one data row per log() call
//   - a non-writable path yields an open() failure and log() becomes a no-op
//
// We use a temp file under the system temp dir and clean it up in TearDown.

#include "twin/GearboxCsvLogger.h"
#include "twin/GearboxLogEntry.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

namespace {

// Build a unique temp path per process using the (non-deprecated) P_tmpdir dir.
std::string makeTempCsvPath() {
    std::string dir = "/tmp";
    if (const char* tmp = std::getenv("TMPDIR")) dir = tmp;
    static int counter = 0;
    return dir + "/gearbox_csv_logger_test_" + std::to_string(++counter) + ".csv";
}

class GearboxCsvLoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Unique temp path per test run.
        path_ = makeTempCsvPath();
    }

    void TearDown() override {
        std::remove(path_.c_str());
    }

    std::string readFile() {
        std::ifstream f(path_);
        std::ostringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    int countLines() {
        const std::string content = readFile();
        if (content.empty()) return 0;
        int lines = 0;
        for (char c : content) {
            if (c == '\n') lines++;
        }
        // File ends with a trailing newline, so lines == rows written.
        return lines;
    }

    int countHeaderOccurrences() {
        const std::string content = readFile();
        const std::string headerMarker = "frame,dt,speedKmh";
        int count = 0;
        size_t pos = 0;
        while ((pos = content.find(headerMarker, pos)) != std::string::npos) {
            count++;
            pos += headerMarker.size();
        }
        return count;
    }

    std::string path_;
};

// Header is written exactly once, and one data row per log() call.
// The logger is destructed (flushing the buffered fprintf) before we read.
TEST_F(GearboxCsvLoggerTest, WritesHeaderOnceAndOneRowPerLog) {
    const int entries = 5;
    {
        twin::GearboxCsvLogger logger(path_);
        for (int i = 0; i < entries; ++i) {
            twin::GearboxLogEntry e;
            e.frame = static_cast<uint64_t>(i);
            e.dt = 0.016;
            logger.log(e);
        }
    }  // logger flushes + closes file here

    // Header line + 5 data rows = 6 lines.
    EXPECT_EQ(countLines(), entries + 1);
    EXPECT_EQ(countHeaderOccurrences(), 1);
}

// Single log writes exactly one header and one data row.
TEST_F(GearboxCsvLoggerTest, SingleLogProducesHeaderAndOneRow) {
    {
        twin::GearboxCsvLogger logger(path_);
        twin::GearboxLogEntry e;
        e.frame = 1;
        logger.log(e);
    }  // logger flushes + closes file here

    EXPECT_EQ(countLines(), 2);
    EXPECT_EQ(countHeaderOccurrences(), 1);
}

// A non-writable path (open fails) makes log() a no-op — no crash, no file.
TEST_F(GearboxCsvLoggerTest, InvalidPathMakesLogNoOp) {
    // A path under a non-existent directory cannot be opened for writing.
    twin::GearboxCsvLogger logger("/nonexistent_directory_xyz/gearbox_log.csv");

    EXPECT_NO_THROW({
        twin::GearboxLogEntry e;
        e.frame = 1;
        logger.log(e);
        logger.log(e);
    });

    // No file should have been created.
    std::ifstream f(path_);
    EXPECT_FALSE(f.good());
}

}  // namespace
