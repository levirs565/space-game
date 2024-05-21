#ifndef SPACE_SCORELISTMANAGER_HPP
#define SPACE_SCORELISTMANAGER_HPP

#include <string>
#include <vector>

namespace ScoreListManager {
struct Score {
  std::string name;
  int score;
};
void addScore(const std::string &name, int score);
std::vector<Score> getList();
}

#endif // SPACE_SCORELISTMANAGER_HPP
