#pragma once

#include "combined/model/material.h"

#include <string>
#include <vector>

namespace wayverb {
namespace combined {
namespace model {
namespace presets {

extern const std::vector<material> materials;

/// Save material presets to a human-editable JSON file.
/// Returns true on success.
bool save_materials_json(const std::vector<material>& mats,
                         const std::string& path);

/// Load material presets from a JSON file.
/// Returns empty vector on failure (caller should fall back to hardcoded).
std::vector<material> load_materials_json(const std::string& path);

}  // namespace presets
}  // namespace model
}  // namespace combined
}  // namespace wayverb
