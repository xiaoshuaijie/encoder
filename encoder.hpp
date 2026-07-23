#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: encoder interface module
constructor_args: []
template_args: []
required_hardware:
  - encoder
  - gpio
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"

class encoder : public LibXR::Application {
public:
  encoder(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  }

  void OnMonitor() override {}

private:
};
