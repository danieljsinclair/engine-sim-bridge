#ifndef TEST_PATH_HELPERS_H
#define TEST_PATH_HELPERS_H

#include "simulator/ScriptLoadHelpers.h"

#include <filesystem>
#include <string>

namespace test_path_helpers {

inline std::string makeAbsolutePath(const std::string& path) {
    return ScriptLoadHelpers::normalizeScriptPath(path);
}

inline std::filesystem::path engineSimRootForAssets(const std::string& assetBase) {
    const std::filesystem::path absoluteAssetBase(makeAbsolutePath(assetBase));
    return absoluteAssetBase.parent_path();
}

class ScopedWorkingDirectory {
public:
    explicit ScopedWorkingDirectory(const std::filesystem::path& path)
        : originalPath_(std::filesystem::current_path()) {
        std::filesystem::current_path(path);
    }

    ~ScopedWorkingDirectory() {
        std::filesystem::current_path(originalPath_);
    }

private:
    std::filesystem::path originalPath_;
};

} // namespace test_path_helpers

#endif // TEST_PATH_HELPERS_H