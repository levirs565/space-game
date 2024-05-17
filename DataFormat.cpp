#include "DataFormat.hpp"
#include <iomanip>

namespace DF {
void trimString(std::string &str) {
  str.erase(0, str.find_first_not_of(" "));
  str.erase(str.find_last_not_of(" ") + 1);
}

Object parseObjectInternal(std::istream &stream, int &lineNumber) {
  Object result;

  std::string line;
  while (std::getline(stream, line)) {
    lineNumber++;
    trimString(line);

    if (line == "}")
      break;

    size_t separatorIndex = line.find_first_of(":");
    if (separatorIndex != std::string::npos) {
      std::string key = line.substr(0, separatorIndex);
      std::string value = line.substr(separatorIndex + 1);

      trimString(key);
      trimString(value);

      if (value == "{") {
        result.field[key] =
            std::make_unique<Object>(parseObjectInternal(stream, lineNumber));
        continue;
      }

      result.field[key] = std::make_unique<String>(value);
      continue;
    }

    if (line.ends_with("{")) {
      if (line.size() == 1) {
        throw ParseError(lineNumber, "Object must bind to builder or field");
      }

      std::string name = line.substr(0, line.size() - 1);

      trimString(name);

      result.builder.push_back(
          std::make_pair(name, parseObjectInternal(stream, lineNumber)));
      continue;
    }

    throw ParseError(lineNumber, "Expected builder or field");
  }

  return result;
}

Object parseObject(std::istream &stream, int initialLineNumber) {
  int lineNumber = initialLineNumber - 1;
  return parseObjectInternal(stream, lineNumber);
}

void printTab(std::ostream &stream, int depth) {
  stream << std::setw(depth * 2) << std::setfill(' ') << "";
}

void serializeObject(Object &object, std::ostream &stream, int depth) {
  if (depth > 0)
    stream << "{" << std::endl;

  for (auto &[key, value] : object.field) {
    printTab(stream, depth);
    stream << key << ": ";

    DF::Value *valuePtr = value.get();

    if (DF::String *stringPtr = dynamic_cast<DF::String *>(valuePtr))
      stream << stringPtr->value << std::endl;
    else if (DF::Object *objectPtr = dynamic_cast<DF::Object *>(valuePtr))
      serializeObject(*objectPtr, stream, depth + 1);
  }

  for (auto &[name, object] : object.builder) {
    printTab(stream, depth);
    stream << name << " ";
    serializeObject(object, stream, depth + 1);
  }

  if (depth > 0) {
    printTab(stream, depth - 1);
    stream << "}" << std::endl;
  }
}
} // namespace DF