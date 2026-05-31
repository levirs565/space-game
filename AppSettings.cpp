#include "AppSettings.hpp"

AppSettings currentAppSettings{.debugFlowField = false,
                               .debugContextSteering = false,
                               .debugBoundingBox = false,
                               .debugMissile = false,
                               .cameraScale = 1.0};

AppSettings *getAppSettings() {
  return &currentAppSettings;
}