/**
 * Source: https://stackoverflow.com/questions/24469927/does-c-have-an-equivalent-to-nets-notimplementedexception
 */
#pragma once

#include <stdexcept>

namespace huron {

class NotImplementedException : public std::logic_error
{
 private:

  std::string _text;

  NotImplementedException(const char* message, const char* function)
      : std::logic_error("Not Implemented") {
    _text = message;
    _text += " : ";
    _text += function;
  };

 public:

  NotImplementedException()
      : NotImplementedException("Not Implememented", __FUNCTION__) {}

  NotImplementedException(const char* message)
      : NotImplementedException(message, __FUNCTION__) {}

  virtual const char *what() const throw() {
      return _text.c_str();
  }
};

}//namespace huron
