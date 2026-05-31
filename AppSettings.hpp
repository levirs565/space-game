#ifndef SPACE_APPSETTINGS_HPP
#define SPACE_APPSETTINGS_HPP

struct AppSettings {
  bool debugFlowField;
  bool debugContextSteering;
  bool debugBoundingBox;
  bool debugMissile;
  double cameraScale;
};

AppSettings* getAppSettings();

#endif //SPACE_APPSETTINGS_HPP
