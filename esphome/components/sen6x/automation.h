#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "sen6x.h"

namespace esphome {
namespace sen6x {

template<typename... Ts> class StartMeasurementAction : public Action<Ts...> {
 public:
  explicit StartMeasurementAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(Ts... x) override { this->sen6x_->start_measurement(); }

 protected:
  SEN6XComponent *sen6x_;
};

template<typename... Ts> class StopMeasurementAction : public Action<Ts...> {
 public:
  explicit StopMeasurementAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(Ts... x) override { this->sen6x_->stop_measurement(); }

 protected:
  SEN6XComponent *sen6x_;
};

}  // namespace sen6x
}  // namespace esphome
