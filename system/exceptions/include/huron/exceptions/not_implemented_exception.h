/**
 * Source: https://stackoverflow.com/questions/24469927/does-c-have-an-equivalent-to-nets-notimplementedexception
 */
#pragma once

#include <stdexcept>
#include <string>

namespace huron {

class InvalidConfigurationException : public std::logic_error {
 private:
  std::string _text;

  InvalidConfigurationException(const char* message, const char* function)
      : std::logic_error("Invalid Configuration provided.") {
    _text = message;
    _text += " : ";
    _text += function;
  }

 public:
  InvalidConfigurationException()
      : InvalidConfigurationException(
            "Invalid Configuration provided.", __FUNCTION__) {}

  explicit InvalidConfigurationException(const char* message)
      : InvalidConfigurationException(message, __FUNCTION__) {}

  virtual const char *what() const throw() {
      return _text.c_str();
  }
};

}  // namespace huron
