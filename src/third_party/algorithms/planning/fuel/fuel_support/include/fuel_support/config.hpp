// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <iostream>
namespace fuel_support {
class Config {
 public:
  YAML::Node values;
  template<class T> void param(const std::string& name,T& out,const T& fallback) const {
    out=values[name]?values[name].as<T>():fallback;
  }
};
struct Stamp {
  double seconds{0};
  static Stamp now(){return {std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()};}
  double toSec() const{return seconds;}
  Stamp operator-(const Stamp& other)const{return {seconds-other.seconds};}
};
}
#define FUEL_WARN(...) do{std::fprintf(stderr,__VA_ARGS__);std::fprintf(stderr,"\n");}while(0)
#define FUEL_INFO(...) FUEL_WARN(__VA_ARGS__)
#define FUEL_ERROR(...) FUEL_WARN(__VA_ARGS__)
#define FUEL_WARN_THROTTLE(period,...) FUEL_WARN(__VA_ARGS__)
#define FUEL_ERROR_COND(condition,...) do{if(condition) FUEL_WARN(__VA_ARGS__);}while(0)
