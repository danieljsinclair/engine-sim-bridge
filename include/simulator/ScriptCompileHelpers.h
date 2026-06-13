#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <iostream>
#include <unistd.h>

namespace script_compile_helpers {

struct ScopedCurrentPath {
    explicit ScopedCurrentPath(const std::filesystem::path& path)
        : originalPath(std::filesystem::current_path()) {
        std::filesystem::current_path(path);
    }

    ~ScopedCurrentPath() {
        std::filesystem::current_path(originalPath);
    }

    std::filesystem::path originalPath;
};

struct ScriptCompileTarget {
    std::filesystem::path simDir;
    std::filesystem::path assetsDir;
    std::filesystem::path compileTarget;
    std::filesystem::path tempScriptPath;
    std::filesystem::path wrapperPath;
    std::string relativeImport;
    bool insideAssets = false;

    ~ScriptCompileTarget() {
        cleanup();
    }

    // Non-copyable to prevent double-cleanup
    ScriptCompileTarget() = default;
    ScriptCompileTarget(const ScriptCompileTarget&) = delete;
    ScriptCompileTarget& operator=(const ScriptCompileTarget&) = delete;
    ScriptCompileTarget(ScriptCompileTarget&& o) noexcept
        : simDir(std::move(o.simDir))
        , assetsDir(std::move(o.assetsDir))
        , compileTarget(std::move(o.compileTarget))
        , tempScriptPath(std::move(o.tempScriptPath))
        , wrapperPath(std::move(o.wrapperPath))
        , relativeImport(std::move(o.relativeImport))
        , insideAssets(o.insideAssets) {
        o.tempScriptPath.clear();
        o.wrapperPath.clear();
    }
    ScriptCompileTarget& operator=(ScriptCompileTarget&& o) noexcept {
        if (this != &o) {
            cleanup();
            simDir = std::move(o.simDir);
            assetsDir = std::move(o.assetsDir);
            compileTarget = std::move(o.compileTarget);
            tempScriptPath = std::move(o.tempScriptPath);
            wrapperPath = std::move(o.wrapperPath);
            relativeImport = std::move(o.relativeImport);
            insideAssets = o.insideAssets;
            o.tempScriptPath.clear();
            o.wrapperPath.clear();
        }
        return *this;
    }

private:
    void cleanup() {
        if (!wrapperPath.empty()) {
            std::filesystem::remove(wrapperPath);
            wrapperPath.clear();
        }
        if (!tempScriptPath.empty()) {
            std::filesystem::remove(tempScriptPath);
            tempScriptPath.clear();
        }
    }
};

inline std::filesystem::path findEngineSimRoot(
    const std::filesystem::path& scriptPath,
    const std::filesystem::path& resolvedAssetPath = {}) {
    namespace fs = std::filesystem;

    fs::path search = fs::absolute(scriptPath).parent_path();
    while (!search.empty()) {
        if (fs::exists(search / "assets") && fs::exists(search / "es" / "engine_sim.mr")) {
            return search;
        }

        if (search == search.parent_path()) break;
        search = search.parent_path();
    }

    if (!resolvedAssetPath.empty()) {
        const fs::path resolvedAssetPathFs(resolvedAssetPath);
        if (fs::exists(resolvedAssetPathFs / "engine_sim.mr")) {
            return resolvedAssetPathFs.parent_path();
        }
        if (fs::exists(resolvedAssetPathFs / "sound-library") &&
            fs::exists(resolvedAssetPathFs.parent_path() / "es" / "engine_sim.mr")) {
            return resolvedAssetPathFs.parent_path();
        }
    }

    throw std::runtime_error("Unable to locate engine-sim root for script: " + fs::absolute(scriptPath).string());
}

inline bool scriptInvokesMain(const std::filesystem::path& scriptPath) {
    std::ifstream in(scriptPath);
    if (!in.is_open()) {
        return false;
    }

    std::ostringstream ss;
    ss << in.rdbuf();
    const std::string content = ss.str();

    // Keep this heuristic simple: if script explicitly invokes main(), do not
    // generate a wrapper that invokes main() again.
    return content.find("main()") != std::string::npos;
}

inline ScriptCompileTarget prepareScriptCompileTarget(
    const std::filesystem::path& scriptPath,
    const std::filesystem::path& simDir) {
    namespace fs = std::filesystem;

    ScriptCompileTarget target;
    target.simDir = simDir;
    target.assetsDir = simDir / "assets";

    const fs::path absScriptPath = fs::absolute(scriptPath);

    if (!fs::exists(target.simDir / "es" / "engine_sim.mr")) {
        throw std::runtime_error("engine_sim.mr not found under: " + (target.simDir / "es").string());
    }

    if (!fs::exists(absScriptPath)) {
        throw std::runtime_error("Script not found: " + absScriptPath.string());
    }

    const fs::path relFromAssets = fs::relative(absScriptPath, target.assetsDir);
    target.relativeImport = relFromAssets.generic_string();
    target.insideAssets = !target.relativeImport.empty() &&
        target.relativeImport[0] != '.' && target.relativeImport[0] != '/';

    if (target.insideAssets) {
        // Some entry scripts (e.g. assets/main.mr) already invoke main().
        // Wrapping those and calling main() again can fail with undefined-node
        // errors in the generated wrapper. Compile directly in that case.
        if (scriptInvokesMain(absScriptPath)) {
            target.compileTarget = absScriptPath;
            return target;
        }

        target.wrapperPath = fs::temp_directory_path() / ("_bridge_wrapper_" + std::to_string(getpid()) + ".mr");
        std::ofstream wrapper(target.wrapperPath);
        if (!wrapper.is_open()) {
            throw std::runtime_error("Cannot create wrapper script: " + target.wrapperPath.string());
        }

        wrapper << "import \"engine_sim.mr\"\n";
        wrapper << "import \"" << target.relativeImport << "\"\n";
        wrapper << "main()\n";
        target.compileTarget = target.wrapperPath;
    } else {
        // create path if it doesn't exist yet
        if (!fs::exists(target.assetsDir)) {
            fs::create_directories(target.assetsDir);
        }
        target.tempScriptPath = target.assetsDir /
            ("_bridge_tmp_" + absScriptPath.stem().string() + "_" + std::to_string(getpid()) + ".mr");
        std::cout << "Copying script from " << absScriptPath << " to: " << target.tempScriptPath << std::endl;
        fs::copy_file(absScriptPath, target.tempScriptPath, fs::copy_options::overwrite_existing);
        target.relativeImport = target.tempScriptPath.filename().generic_string();
        target.compileTarget = target.tempScriptPath;
    }

    return target;
}

// Cleanup is handled automatically by ScriptCompileTarget's destructor (RAII).
// This function is retained for backwards compatibility with existing callers.
inline void cleanupScriptCompileTarget(const ScriptCompileTarget& /*target*/) {}

}  // namespace script_compile_helpers