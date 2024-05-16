#ifndef DATAFORMAT_HPP_
#define DATAFORMAT_HPP_

#include <map>
#include <vector>
#include <memory>

namespace DF {
    struct Value {
        virtual ~Value() = default;
    };

    struct Object : Value {
        std::map<std::string, std::unique_ptr<Value>> field;
        std::vector<std::pair<std::string, Object>> builder;
    };

    struct String : Value {
        std::string value;
        
        String(const std::string& value): value{value} {
        }
    };

    class ParseError : public std::exception {
        std::string mWhat;
    public:
        ParseError(int lineNumber, const std::string reason) {
            mWhat = "Error at line " + std::to_string(lineNumber) + " : " + reason;
        }
        
        const char* what() const noexcept override
        {
            return mWhat.c_str();
        }
    };

    void serializeObject(Object &object, std::ostream &stream, int depth = 0);
    Object parseObject(std::istream &stream, int initialLineNumber = 1);
}


#endif // DATAFORMAT_HPP_