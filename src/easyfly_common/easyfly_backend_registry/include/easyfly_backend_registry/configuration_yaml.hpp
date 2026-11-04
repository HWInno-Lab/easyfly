// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_backend_registry/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#pragma once
#include <regex>
#include <yaml-cpp/yaml.h>
#include "easyfly_backend_registry/metadata.hpp"
namespace easyfly::backends {
inline Parameters parameters_from_yaml(const YAML::Node & node, const std::string & capability, const std::string & backend) {
  Parameters result;
  if (!node || node.IsNull()) return result;
  if (!node.IsMap() || node.size() > 128) throw std::invalid_argument("backend_parameters must be a bounded mapping");
  const auto & schema = parameter_schema(capability, backend);
  for (const auto & pair : node) {
    const auto name = pair.first.as<std::string>(); const auto & value = pair.second;
    const Field * spec = nullptr;
    for (const auto & field : schema.fields) if (name == field.name) {spec = &field; break;}
    if (!spec || result.count(name)) throw std::invalid_argument("unknown or duplicate backend parameter: " + name);
    const std::string type = spec->type;
    if (type == "array") {
      if (!value.IsSequence() || value.size() > spec->max_size) throw std::invalid_argument("invalid backend array: " + name);
      result[name] = value.as<std::vector<std::string>>();
    } else {
      if (!value.IsScalar()) throw std::invalid_argument("backend scalar required: " + name);
      const auto text = value.Scalar();
      if (type == "string") result[name] = text;
      else {
        if (value.Tag() == "!" || value.Tag() == "tag:yaml.org,2002:str") throw std::invalid_argument("quoted backend number/bool: " + name);
        if (type == "boolean") {
          if (text != "true" && text != "false") throw std::invalid_argument("backend bool must be true/false: " + name);
          result[name] = text == "true";
        } else if (type == "integer") {
          if (!std::regex_match(text, std::regex("[-+]?[0-9]+"))) throw std::invalid_argument("backend integer required: " + name);
          std::size_t used; const auto v = std::stoll(text, &used); if (used != text.size()) throw std::invalid_argument(name);
          result[name] = static_cast<std::int64_t>(v);
        } else {
          std::size_t used; const double v = std::stod(text, &used); if (used != text.size()) throw std::invalid_argument(name);
          result[name] = v;
        }
      }
    }
    validate_value(*spec, result.at(name));
  }
  return result;
}
inline void overlay(Parameters & base, const Parameters & higher) {for (const auto & p : higher) base[p.first] = p.second;}
}  // namespace easyfly::backends
