// AssetResolver.h - Exe-aware resolution for runtime-relative asset paths
//
// Relative asset references (scripts, sound-library WAVs) must work no matter
// which directory the binary was launched from. The resolution order is:
//
//   1. the executable's real (symlink-resolved) directory  (<repo>/build/)
//   2. its parent — the repository root for in-tree builds (<repo>/)
//   3. the current working directory                        (legacy fallback)
//
// This mirrors the CLI's config/ExecutablePath behaviour but lives in the
// bridge so the simulator's own loaders (script paths, impulse-response WAVs)
// resolve identically in every host application.

#ifndef ENGINE_SIM_BRIDGE_ASSET_RESOLVER_H
#define ENGINE_SIM_BRIDGE_ASSET_RESOLVER_H

#include <algorithm>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#elif defined(__linux__)
#include <limits.h>
#include <unistd.h>
#endif

namespace asset_resolver {

// Real (canonical) directory containing the running executable. Resolves
// symlinks and ".." components so a binary invoked through a symlinked or
// relative path still anchors to its true location. Returns an empty path
// when the platform lookup fails.
inline std::filesystem::path executableDirectory() {
    namespace fs = std::filesystem;
    std::string raw;

#if defined(__APPLE__)
    std::vector<char> buf(1024);
    uint32_t size = static_cast<uint32_t>(buf.size());
    if (_NSGetExecutablePath(buf.data(), &size) != 0) {
        buf.resize(size + 1);
        size = static_cast<uint32_t>(buf.size());
        if (_NSGetExecutablePath(buf.data(), &size) != 0) return {};
    }
    raw = buf.data();
#elif defined(__linux__)
    std::vector<char> buf(PATH_MAX);
    const ssize_t len = readlink("/proc/self/exe", buf.data(), buf.size() - 1);
    if (len <= 0) return {};
    buf[static_cast<size_t>(len)] = '\0';
    raw = buf.data();
#else
    return {};
#endif

    std::error_code ec;
    const fs::path real = fs::weakly_canonical(raw, ec);
    if (ec || real.empty()) return fs::path(raw).lexically_normal().parent_path();
    return real.parent_path();
}

// Search roots in resolution order: executable directory, its parent (the
// repository root for in-tree builds such as <repo>/build/<exe>), then the
// current working directory. Roots are canonicalized, de-duplicated and
// limited to directories that exist.
inline std::vector<std::filesystem::path> searchRoots() {
    namespace fs = std::filesystem;
    std::vector<fs::path> roots;

    const fs::path exeDir = executableDirectory();
    if (!exeDir.empty()) {
        roots.push_back(exeDir);
        const fs::path parent = exeDir.parent_path();
        if (!parent.empty()) roots.push_back(parent);
    }

    std::error_code ec;
    if (const fs::path cwd = fs::current_path(ec); !ec && !cwd.empty()) {
        roots.push_back(cwd);
    }

    std::vector<fs::path> ordered;
    for (const auto& root : roots) {
        std::error_code existsEc;
        if (!fs::exists(root, existsEc) || existsEc) continue;

        std::error_code canonEc;
        fs::path canonical = fs::weakly_canonical(root, canonEc);
        if (canonEc) canonical = root.lexically_normal();

        const bool seen = std::any_of(ordered.begin(), ordered.end(),
            [&canonical](const fs::path& kept) {
                std::error_code equivEc;
                return fs::equivalent(kept, canonical, equivEc) && !equivEc;
            });
        if (!seen) ordered.push_back(canonical);
    }
    return ordered;
}

// Every probe for `relative`, in resolution order: `relative` joined under
// each root. Absolute (or empty) references pass through unchanged as the
// single candidate. When `roots` is empty the standard searchRoots() chain is
// used. Exposed so error paths can list every location tried.
inline std::vector<std::filesystem::path> candidatesFor(
    const std::filesystem::path& relative,
    const std::vector<std::filesystem::path>& roots = {})
{
    namespace fs = std::filesystem;
    std::vector<fs::path> probes;

    if (relative.empty() || relative.is_absolute()) {
        probes.push_back(relative);
        return probes;
    }

    const fs::path normalized = relative.lexically_normal();
    const std::vector<fs::path> search = roots.empty() ? searchRoots() : roots;
    for (const auto& root : search) {
        if (root.empty()) continue;
        probes.push_back(root / normalized);
    }

    if (probes.empty()) {
        std::error_code ec;
        if (const fs::path cwd = fs::current_path(ec); !ec) {
            probes.push_back(cwd / normalized);
        } else {
            probes.push_back(normalized);
        }
    }
    return probes;
}

// Resolve `relative` to the first existing probe (exe dir -> repo root ->
// cwd). When nothing exists, the first probe is returned so callers produce a
// concrete "not found" path instead of an empty one. Absolute references are
// returned unchanged.
inline std::filesystem::path resolve(
    const std::filesystem::path& relative,
    const std::vector<std::filesystem::path>& roots = {})
{
    namespace fs = std::filesystem;
    if (relative.is_absolute()) return relative;

    const std::vector<fs::path> probes = candidatesFor(relative, roots);
    for (const auto& candidate : probes) {
        std::error_code ec;
        if (fs::exists(candidate, ec) && !ec) {
            return candidate.lexically_normal();
        }
    }
    return probes.empty() ? relative : probes.front().lexically_normal();
}

} // namespace asset_resolver

#endif // ENGINE_SIM_BRIDGE_ASSET_RESOLVER_H
