#ifndef SPACE_DATAFORMAT_HPP
#define SPACE_DATAFORMAT_HPP

#include <map>
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>

namespace DF {
struct Value {
  virtual ~Value() = default;
};

struct Object : Value {
  std::map<std::string, std::unique_ptr<Value>> field;
  std::vector<std::pair<std::string, Object>> builder;

  template <class Type>
  Type* getField(const std::string& name) {
    if (!field.contains(name))
      throw std::runtime_error("Field not found: " + name);
    auto result = dynamic_cast<Type*>(field[name].get());
    if (result == nullptr)
      throw std::runtime_error("Field has invalid value: " + name);
    return result;
  }
  
  Object() {}
  Object(const Object&) = delete;
  Object(Object&&) = default;
  Object& operator=(Object&&) = default;
};

struct String : Value {
  std::string value;

  String(const std::string &value) : value{value} {}
};

class ParseError : public std::exception {
  std::string mWhat;

public:
  ParseError(int lineNumber, const std::string reason) {
    mWhat = "Error at line " + std::to_string(lineNumber) + " : " + reason;
  }

  const char *what() const noexcept override { return mWhat.c_str(); }
};

void serializeObject(Object &object, std::ostream &stream, int depth = 0);
Object parseObject(std::istream &stream, int initialLineNumber = 1);
} // namespace DF

#endif /* SPACE_DATAFORMAT_HPP */
