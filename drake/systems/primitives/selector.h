#pragma once

#include <memory>
#include <unordered_map>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// This system splits a vector valued signal in its inputs of size `size`
/// into `size` output scalar valued signals.
/// The input to this system directly feeds through to its output.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class Selector : public LeafSystem<T> {
 public:
  /// Constructs %Selector with one vector valued input port of size
  /// @p size and vector valued output ports of size @p output_ports_sizes.
  ///
  /// @p output_ports_sizes must exactly divide @p size. Otherwise this
  /// constructor throws an exception. The number of output ports is therefore
  /// `size / output_ports_sizes`.
  ///
  /// @param size is the size of the input signal to be demultiplexed into its
  /// individual components.
  /// @param output_ports_sizes The size of the output ports. @p length must be
  /// a multiple of @p output_ports_sizes.
  explicit Selector(int input_size, const std::vector<int>& input_indices);

  const std::unordered_map<int, int>& get_input_to_output() const { return input_to_output_; }
  const std::unordered_map<int, int>& get_output_to_input() const { return output_to_input_; }

 private:
  std::unordered_map<int, int> input_to_output_;
  std::unordered_map<int, int> output_to_input_;
  // Sets the i-th output port to the value of the i-th component of the input
  // port.
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;
};

}  // namespace systems
}  // namespace drake
