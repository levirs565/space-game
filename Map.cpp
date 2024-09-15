#include "Map.hpp"
#include "AssetManager.hpp"
#include "DataFormat.hpp"
#include "Entity/Meteor.hpp"
#include <filesystem>
#include <fstream>

Map Map::fromAsset() {
  Map map;

  std::filesystem::path dataFile =
      AssetManager::getInstance()->getAsset("Map.data");

  std::ifstream dataStream;
  dataStream.open(dataFile);

  if (!dataStream.is_open())
    throw std::runtime_error("Cannot open map");

  DF::Object object = std::move(DF::parseObject(dataStream));

  auto *widthValue = object.getField<DF::String>("width");
  auto *heightValue = object.getField<DF::String>("height");

  map.size.x = std::stod(widthValue->value);
  map.size.y = std::stod(heightValue->value);

  auto entityList = object.getField<DF::Object>("entityList");

  for (auto &[name, entity] : entityList->builder) {
    auto entityXValue = entity.getField<DF::String>("x");
    auto entityYValue = entity.getField<DF::String>("y");
    Vec2 position{std::stod(entityXValue->value),
                  std::stod(entityYValue->value)};
    if (name == "Meteor") {
      auto entityTypeValue = entity.getField<DF::String>("type");
      map.entityList.push_back(
          std::move(std::make_unique<Meteor>(position, entityTypeValue->value)));
    } else {
      throw std::runtime_error("Undefined entity type: " + name);
    }
  }

  return map;
}
