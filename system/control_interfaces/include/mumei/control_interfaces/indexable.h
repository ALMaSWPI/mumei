#pragma once

#include <string>
#include <cstdint>

namespace mumei {

typedef uint32_t Index;
const Index kInvalidIndex = -1;

class Indexable {
 public:
  explicit Indexable(const std::string& name)
      : name_(name), index_(kInvalidIndex) {}
  virtual ~Indexable() = default;

  /**
   * Set the index of the object.
   * @note This method should not be called manually by the user.
   * @param index The index to be set.
   */
  void SetIndex(Index index) { index_ = index; }

  Index index() const { return index_; }
  const std::string& name() const { return name_; }

  bool IsRegistered() const { return index_ != kInvalidIndex; }

 private:
  std::string name_;
  Index index_;
};

}  // namespace mumei
