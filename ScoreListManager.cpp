#include "ScoreListManager.hpp"
#include "DataFormat.hpp"
#include <fstream>
#include <optional>
#include <algorithm>

std::string scoreListFileName = "score-list.data";

std::optional<DF::Object> parseScoreList() {
  std::ifstream stream;
  stream.open(scoreListFileName, std::ios::in);
  if (!stream.is_open())
    return std::nullopt;

  return DF::parseObject(stream);
}

DF::Object createBlankScoreList() {
  DF::Object obj;
  obj.field["scoreList"] = std::move(std::make_unique<DF::Object>());
  return obj;
}

void saveScoreList(DF::Object &object) {
  std::ofstream stream;
  stream.open(scoreListFileName, std::ios::out);
  if (!stream.is_open())
    throw std::runtime_error("cannot save score list");

  DF::serializeObject(object, stream);
}

void ScoreListManager::addScore(const std::string &name, int score) {
  std::optional<DF::Object> savedScore = parseScoreList();
  DF::Object currentScoreList = savedScore.has_value()
                                    ? std::move(savedScore.value())
                                    : std::move(createBlankScoreList());

  DF::Object scoreObject;
  scoreObject.field["name"] = std::move(std::make_unique<DF::String>(name));
  scoreObject.field["score"] =
      std::move(std::make_unique<DF::String>(std::to_string(score)));

  auto *scoreListObject = currentScoreList.getField<DF::Object>("scoreList");

  scoreListObject->builder.emplace_back("Score", std::move(scoreObject));

  std::sort(scoreListObject->builder.begin(), scoreListObject->builder.end(),
            [](auto &a, auto &b) {
              int aScore = std::stoi(
                  a.second.template getField<DF::String>("score")->value);
              int bScore = std::stoi(
                  b.second.template getField<DF::String>("score")->value);
              return aScore > bScore;
            });

  if (scoreListObject->builder.size() > 10)
    scoreListObject->builder.erase(scoreListObject->builder.begin() + 10,
                                   scoreListObject->builder.end());

  saveScoreList(currentScoreList);
}
std::vector<ScoreListManager::Score> ScoreListManager::getList() {
  std::vector<Score> result;

  std::optional<DF::Object> savedScore = parseScoreList();

  if (!savedScore.has_value())
    return result;

  DF::Object currentScoreList = std::move(savedScore.value());

  auto *scoreListObject = currentScoreList.getField<DF::Object>("scoreList");

  for (auto &scoreObject : scoreListObject->builder) {
    result.push_back(
        {.name = scoreObject.second.getField<DF::String>("name")->value,
         .score = std::stoi(
             scoreObject.second.getField<DF::String>("score")->value)});
  }

  return result;
}
