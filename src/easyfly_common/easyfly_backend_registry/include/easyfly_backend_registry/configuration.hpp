// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <sstream>
#include <iomanip>
#include <locale>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>
namespace easyfly::backends {
using Value = std::variant<bool, std::int64_t, double, std::string, std::vector<std::string>>;
using Parameters = std::map<std::string, Value>;
struct BackendConfig {std::string backend; Parameters parameters; unsigned contract_version{1};};
struct Field {
  const char * name; const char * type;
  double minimum, maximum; std::size_t min_size, max_size, item_max_size;
  std::optional<Value> default_value; bool required;
};
struct Schema {const char * capability; const char * backend; std::vector<Field> fields;};
inline void validate_value(const Field & field, const Value & value) {
  const std::string type = field.type;
  bool valid = false;
  if (type == "integer" || type == "number") {
    std::optional<double> numeric;
    if (const auto p = std::get_if<std::int64_t>(&value)) numeric = static_cast<double>(*p);
    if (type == "number") {if (const auto p = std::get_if<double>(&value)) numeric = *p;}
    valid = numeric && std::isfinite(*numeric) && *numeric >= field.minimum && *numeric <= field.maximum;
  } else if (type == "boolean") valid = std::holds_alternative<bool>(value);
  else if (type == "string") {
    if (const auto p = std::get_if<std::string>(&value)) valid = p->size() >= field.min_size && p->size() <= field.max_size;
  } else if (type == "array") {
    if (const auto p = std::get_if<std::vector<std::string>>(&value)) {
      valid = p->size() >= field.min_size && p->size() <= field.max_size;
      for (const auto & item : *p) valid = valid && item.size() <= field.item_max_size;
    }
  }
  if (!valid) throw std::invalid_argument(std::string("invalid backend parameter: ") + field.name);
}
inline Parameters resolve(const Schema & schema, const Parameters & input) {
  Parameters result;
  for (const auto & field : schema.fields) if (field.default_value) result.emplace(field.name, *field.default_value);
  for (const auto & entry : input) {
    const Field * found = nullptr;
    for (const auto & field : schema.fields) if (entry.first == field.name) {found = &field; break;}
    if (!found) throw std::invalid_argument("unknown backend parameter: " + entry.first);
    validate_value(*found, entry.second); result[entry.first] = entry.second;
  }
  for (const auto & field : schema.fields) {
    const auto it = result.find(field.name);
    if (it == result.end()) {if (field.required) throw std::invalid_argument(std::string("missing backend parameter: ") + field.name);}
    else validate_value(field, it->second);
  }
  return result;
}
template<class T> T parameter(const Parameters & values, const std::string & name) {
  const auto & value = values.at(name);
  if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
    const auto p = std::get_if<std::int64_t>(&value);
    if (!p || static_cast<long double>(*p) < std::numeric_limits<T>::lowest() ||
        static_cast<long double>(*p) > std::numeric_limits<T>::max()) throw std::invalid_argument("integer parameter out of range: " + name);
    return static_cast<T>(*p);
  } else if constexpr (std::is_same_v<T, double>) {
    if (const auto p = std::get_if<std::int64_t>(&value)) return static_cast<double>(*p);
    return std::get<double>(value);
  } else return std::get<T>(value);
}
// Construct then freeze a local table. No process-global mutation or plugin loading.
template<class Product> class Factory {
public:
  using Creator = std::function<std::unique_ptr<Product>(const Parameters &)>;
  using Normalizer = std::function<Parameters(const Parameters &)>;
  void add(std::string id, Creator creator, Normalizer normalize = {}) {
    if (id.empty() || !creator || !creators_.emplace(std::move(id), Entry{std::move(creator), std::move(normalize)}).second)
      throw std::invalid_argument("empty or duplicate backend creator");
  }
  BackendConfig resolve(const BackendConfig & config) const {
    if (config.contract_version != 1) throw std::invalid_argument("unsupported backend contract version");
    const auto it = creators_.find(config.backend);
    if (it == creators_.end()) throw std::invalid_argument("unknown backend creator: " + config.backend);
    return {config.backend, it->second.normalize ? it->second.normalize(config.parameters) : config.parameters, config.contract_version};
  }
  std::unique_ptr<Product> create(const BackendConfig & config) const {
    const auto effective = resolve(config);
    auto result = creators_.at(config.backend).create(effective.parameters);
    if (!result) throw std::runtime_error("backend creator returned null: " + config.backend);
    return result;
  }
private:
  struct Entry {Creator create; Normalizer normalize;};
  std::map<std::string, Entry> creators_;
};
inline std::string json_string(const std::string & value) {
  std::ostringstream out; out << '"';
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c) << std::dec;
    else out << c;
  }
  out << '"'; return out.str();
}
inline std::string parameters_json(const Parameters & parameters) {
  std::ostringstream out; out.imbue(std::locale::classic()); out << '{'; bool first = true;
  for (const auto & item : parameters) {
    if (!first) {out << ',';}
    first = false; out << json_string(item.first) << ':';
    std::visit([&](const auto & value) {
      using T = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<T, std::string>) out << json_string(value);
      else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
        out << '['; bool start = true; for (const auto & v : value) {if (!start) out << ','; start = false; out << json_string(v);} out << ']';
      } else if constexpr (std::is_same_v<T, bool>) out << (value ? "true" : "false");
      else {if constexpr (std::is_same_v<T, double>) {if (!std::isfinite(value)) throw std::invalid_argument("non-finite JSON");} out << std::setprecision(17) << value;}
    }, item.second);
  }
  out << '}'; return out.str();
}
}  // namespace easyfly::backends
