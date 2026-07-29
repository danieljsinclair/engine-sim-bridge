#pragma once

#include "compiler.h"
#include "simulator/ScriptCompileHelpers.h"

#include <filesystem>
#include <stdexcept>
#include <string>

namespace script_execution_helpers {

inline es_script::Compiler::Output compileScript(
    const std::filesystem::path& scriptPath,
    const std::filesystem::path& simDir)
{
    auto compileTarget = script_compile_helpers::prepareScriptCompileTarget(scriptPath, simDir);
    script_compile_helpers::ScopedCurrentPath scopedCurrentPath(compileTarget.simDir);

    es_script::Compiler compiler;
    compiler.initialize();

    try {
        if (!compiler.compile(compileTarget.compileTarget.string().c_str())) {
            throw std::runtime_error("Failed to compile script: " + std::filesystem::absolute(scriptPath).string());
        }

        es_script::Compiler::Output output = compiler.execute();
        script_compile_helpers::cleanupScriptCompileTarget(compileTarget);
        compiler.destroy();
        return output;
    } catch (...) {
        script_compile_helpers::cleanupScriptCompileTarget(compileTarget);
        compiler.destroy();
        throw;
    }
}

}  // namespace script_execution_helpers
