#include "core/program_wrapper.h"

#include <iostream>

namespace wayverb {
namespace core {

program_wrapper::program_wrapper(const compute_context& cc,
                                 const std::string& source)
        : program_wrapper(cc, std::make_pair(source.data(), source.size())) {}

program_wrapper::program_wrapper(const compute_context& cc,
                                 const std::pair<const char*, size_t>& source)
        : program_wrapper(
                  cc, std::vector<std::pair<const char*, size_t>>{source}) {}

program_wrapper::program_wrapper(const compute_context& cc,
                                 const std::vector<std::string>& sources)
        : program_wrapper(cc, [&sources] {
            std::vector<std::pair<const char*, size_t>> ret;
            ret.reserve(sources.size());
            for (const auto& source : sources) {
                ret.emplace_back(std::make_pair(source.data(), source.size()));
            }
            return ret;
        }()) {}

program_wrapper::program_wrapper(
        const compute_context& cc,
        const std::vector<std::pair<const char*, size_t>>& sources)
        : device(cc.device)
        , program(cc.context, [&] {
              // opencl.hpp v2 Sources = vector<string>; v1 used vector<pair<const char*,size_t>>
              std::vector<std::string> strs;
              strs.reserve(sources.size());
              for (const auto& s : sources) {
                  strs.emplace_back(s.first, s.second);
              }
              return strs;
          }()) {
    build(device);
}

void program_wrapper::build(const cl::Device& device) const {
    try {
        program.build({device}, "");
    } catch (const cl::Error& e) {
        const auto log = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device);
        fprintf(stderr, "[cl] build FAILED (%d): %s\n[cl] log:\n%s\n",
                e.err(), e.what(), log.c_str());
        fflush(stderr);
        throw;
    }
    // Print any warnings even on success
    const auto log = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device);
    if (!log.empty()) {
        fprintf(stderr, "[cl] build log:\n%s\n", log.c_str());
        fflush(stderr);
    }
}

cl::Device program_wrapper::get_device() const { return device; }

}  // namespace core
}  // namespace wayverb
