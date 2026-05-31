#include "AppSettings.hpp"

#include "DataFormat.hpp"

#include <fstream>
#include <optional>
#include <string>

AppSettings currentAppSettings{
    .debugFlowField = false,
    .debugContextSteering = false,
    .debugBoundingBox = false,
    .debugMissile = false,
    .cameraScale = 1.0,
    .lastShipColor = "red",
    .lastShip = 1,
    .lastMode = 2,
};
std::string appSettingsFileName = "app-settings.data";
std::string debugFlowFieldKey = "debugFlowField";
std::string debugContextSteeringKey = "debugContextSteering";
std::string debugBoundingBoxKey = "debugBoundingBox";
std::string debugMissileKey = "debugMissile";
std::string cameraScaleKey = "cameraScale";
std::string lastShipColorKey = "lastShipColor";
std::string lastShipKey = "lastShip";
std::string lastModeKey = "lastMode";
std::string trueValue = "true";
std::string falseValue = "false";
bool isAppSettingsInitialized{false};

std::optional<DF::Object> parseAppSettings() {
  std::ifstream stream;
  stream.open(appSettingsFileName, std::ios::in);
  if (!stream.is_open())
    return std::nullopt;

  return DF::parseObject(stream);
}

std::unique_ptr<DF::String> mapBooleanToString(bool value) {
  std::string raw = value ? trueValue : falseValue;
  return std::make_unique<DF::String>(raw);
}

AppSettings *getAppSettings() {
  if (!isAppSettingsInitialized) {
    auto object = parseAppSettings();
    if (object.has_value()) {
      auto value = std::move(object.value());
      auto debugFlowField =
          value.getField<DF::String>(debugFlowFieldKey)->value == trueValue;
      auto debugContextSteering =
          value.getField<DF::String>(debugContextSteeringKey)->value ==
          trueValue;
      auto debugBoundingBox =
          value.getField<DF::String>(debugBoundingBoxKey)->value == trueValue;
      auto debugMissile =
          value.getField<DF::String>(debugMissileKey)->value == trueValue;
      auto cameraScale =
          std::stod(value.getField<DF::String>(cameraScaleKey)->value);
      auto lastShipColor = value.getField<DF::String>(lastShipColorKey)->value;
      auto lastShip = std::stoi(value.getField<DF::String>(lastShipKey)->value);
      auto lastMode = std::stoi(value.getField<DF::String>(lastModeKey)->value);

      currentAppSettings.debugFlowField = debugFlowField;
      currentAppSettings.debugContextSteering = debugContextSteering;
      currentAppSettings.debugBoundingBox = debugBoundingBox;
      currentAppSettings.debugMissile = debugMissile;
      currentAppSettings.cameraScale = cameraScale;
      currentAppSettings.lastShipColor = lastShipColor;
      currentAppSettings.lastShip = lastShip;
      currentAppSettings.lastMode = lastMode;
    }
  }

  return &currentAppSettings;
}
void saveAppSettings() {
  std::optional<DF::Object> savedSettings = parseAppSettings();
  DF::Object currentObject = savedSettings.has_value()
                                 ? std::move(savedSettings.value())
                                 : std::move(DF::Object());

  currentObject.field[debugFlowFieldKey] =
      mapBooleanToString(currentAppSettings.debugFlowField);
  currentObject.field[debugContextSteeringKey] =
      mapBooleanToString(currentAppSettings.debugContextSteering);
  currentObject.field[debugBoundingBoxKey] =
      mapBooleanToString(currentAppSettings.debugBoundingBox);
  currentObject.field[debugMissileKey] =
      mapBooleanToString(currentAppSettings.debugMissile);
  currentObject.field[cameraScaleKey] = std::make_unique<DF::String>(
      std::to_string(currentAppSettings.cameraScale));
  currentObject.field[lastShipColorKey] =
      std::make_unique<DF::String>(currentAppSettings.lastShipColor);
  currentObject.field[lastShipKey] =
      std::make_unique<DF::String>(std::to_string(currentAppSettings.lastShip));
  currentObject.field[lastModeKey] =
      std::make_unique<DF::String>(std::to_string(currentAppSettings.lastMode));

  std::ofstream stream;
  stream.open(appSettingsFileName, std::ios::out);
  if (!stream.is_open())
    throw std::runtime_error("cannot save score list");

  DF::serializeObject(currentObject, stream);
}