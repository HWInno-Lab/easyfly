// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
// Execute the actual native Sensor::NeedsUpdate with a controlled scene clock.
// Only the input clock is substituted; no implementation of scheduling is mocked.
#include <gazebo/rendering/Scene.hh>
#include <gazebo/sensors/Sensor.hh>
#include <iostream>
#include <dlfcn.h>
#include <stdexcept>

namespace {gazebo::common::Time scene_time;}
namespace gazebo { namespace rendering {
common::Time Scene::SimTime() const {return scene_time;}
}}
class SensorProbe : public gazebo::sensors::Sensor
{
public:
  SensorProbe() : Sensor(gazebo::sensors::IMAGE)
  {
    scene.reset(new gazebo::rendering::Scene("clock_contract", true));
    updatePeriod = gazebo::common::Time(0.03);
    useStrictRate = false;
  }
  void prime(double value)
  {
    lastMeasurementTime = gazebo::common::Time(value);
    lastUpdateTime = lastMeasurementTime;
  }
  bool due() {return NeedsUpdate();}
  double measured() const {return lastMeasurementTime.Double();}
  double updated() const {return lastUpdateTime.Double();}
};
void require(bool value, const char *message)
{
  if (!value) {throw std::runtime_error(message);}
}
int main()
{
  try {
    Dl_info native{};
    require(dladdr(dlsym(RTLD_DEFAULT, "_ZN6gazebo7sensors6Sensor11NeedsUpdateEv"),
      &native) != 0, "native scheduler library not found");
    std::cout << "scheduler library: " << native.dli_fname << "\n";
    SensorProbe sensor;
    sensor.prime(1.0);
    scene_time = gazebo::common::Time(1.0);
    for (int i = 0; i < 10; ++i) {
      require(!sensor.due(), "equal scene time scheduled another image");
      require(sensor.measured() == 1.0 && sensor.updated() == 1.0,
        "equal scene time erased sampling history");
    }
    scene_time = gazebo::common::Time(1.01);
    require(!sensor.due(), "sampling period bypassed");
    scene_time = gazebo::common::Time(1.04);
    require(sensor.due(), "advancing scene time did not resume sampling");
    scene_time = gazebo::common::Time(0.5);
    require(!sensor.due(), "rollback must reset before sampling");
    require(sensor.measured() == 0.0 && sensor.updated() == 0.0,
      "true rollback failed to reset sampling history");
    require(sensor.due(), "new epoch did not resume sampling");
    sensor.prime(0.0);
    scene_time = gazebo::common::Time(0.0);
    require(!sensor.due(), "zero-time initialization unexpectedly sampled");
    std::cout << "native sensor clock: equal / period / advance / rollback / zero passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
