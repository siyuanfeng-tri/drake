#pragma once

#include <memory>
#include <stdexcept>
#include <utility>

namespace drake {
namespace systems {

template <typename CloneOnlyType>
class CloneOnlyValue : public Value<CloneOnlyType*> {
 public:
  CloneOnlyValue(std::unique_ptr<CloneOnlyType> v)
      : Value<CloneOnlyType*>(v.get()), owned_value_(std::move(v)) {
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  ~CloneOnlyValue() override {}

  explicit CloneOnlyValue(const CloneOnlyValue& other)
      : Value<CloneOnlyType*>(nullptr) {
    if (other.get_value() != nullptr) {
      owned_value_ = other.get_value()->Clone();
      this->set_value(owned_value_.get());
    }
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  CloneOnlyValue& operator=(const CloneOnlyValue& other) {
    if (this == &other) {
      // Special case to do nothing, to avoid an unnecessary Clone.
    } else if (other.get_value() == nullptr) {
      owned_value_.reset();
      this->set_value(owned_value_.get());
    } else {
      owned_value_ = other.get_value()->Clone();
      this->set_value(owned_value_.get());
    }
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  std::unique_ptr<AbstractValue> Clone() const override {
    return std::make_unique<CloneOnlyValue>(*this);
  }

 private:
  void CheckInvariants() {
    DRAKE_DEMAND(owned_value_.get() == this->get_value());
  }

  std::unique_ptr<CloneOnlyType> owned_value_;
};

}  // namespace systems
}  // namespace drake
