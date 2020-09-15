#ifndef CGAL_DEFINITIONS_EXCEPTIONS_H_
#define CGAL_DEFINITIONS_EXCEPTIONS_H_
#include <stdexcept>
/*
 * Simple NotImplementedException as seen on
 * https://stackoverflow.com/questions/24469927
 */
namespace cad_percept {
class NotImplementedException : public std::logic_error {
 private:
  std::string _text;

  NotImplementedException(const char* message, const char* function)
      : std::logic_error("Not Implemented") {
    _text = message;
    _text += " : ";
    _text += function;
  };

 public:
  NotImplementedException() : NotImplementedException("Not Implememented", __FUNCTION__) {}

  NotImplementedException(const char* message) : NotImplementedException(message, __FUNCTION__) {}

  virtual const char* what() const throw() { return _text.c_str(); }
};
}  // namespace cad_percept
#endif  // CGAL_DEFINITIONS_EXCEPTIONS_H_
