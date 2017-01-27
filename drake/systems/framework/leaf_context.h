#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// %LeafContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a leaf System. Specifically, a
/// %LeafContext contains and owns the State, and also contains (but does not
/// own) pointers to the value sources for Inputs, as well as the simulation
/// time and the cache.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
// TODO(david-german-tri): Manage cache invalidation.
template <typename T>
class LeafContext : public Context<T> {
 public:
  // LeafContext objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafContext)

  LeafContext() : state_(std::make_unique<State<T>>()) {}
  ~LeafContext() override {}

  void SetInputPort(int index, std::unique_ptr<InputPort> port) override {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    // TODO(david-german-tri): Set invalidation callbacks.
    inputs_[index] = std::move(port);
  }

  /// Removes all the input ports, and deregisters them from the output ports
  /// on which they depend.
  void ClearInputPorts() { inputs_.clear(); }

  /// Clears the input ports and allocates @p n new input ports, not connected
  /// to anything.
  void SetNumInputPorts(int n) {
    ClearInputPorts();
    inputs_.resize(n);
  }

  int get_num_input_ports() const override {
    return static_cast<int>(inputs_.size());
  }

  const State<T>& get_state() const override { return *state_; }

  State<T>* get_mutable_state() override { return state_.get(); }

  /// Reserves a cache entry with the given @p prerequisites on which it
  /// depends. Returns a ticket to identify the entry.
  CacheTicket CreateCacheEntry(
      const std::set<CacheTicket>& prerequisites) const {
    // TODO(david-german-tri): Provide a notation for specifying context
    // dependencies as well, and provide automatic invalidation when the
    // context dependencies change.
    return cache_.MakeCacheTicket(prerequisites);
  }

  /// Stores the given @p value in the cache entry for the given @p ticket,
  /// and returns a bare pointer to @p value.  That pointer will be invalidated
  /// whenever any of the @p ticket's declared prerequisites change, and
  /// possibly also at other times which are not defined.
  ///
  /// Systems MUST NOT depend on a particular value being present or valid
  /// in the Cache, and MUST check the validity of cached values using
  /// the GetCachedValue interface.
  //
  /// The Cache is useful to avoid recomputing expensive intermediate data. It
  /// is not a scratch space for arbitrary state. If you cannot derive a value
  /// from other fields in the Context, do not put that value in the Cache.
  /// If you violate this rule, you may be devoured by a horror from another
  /// universe, and forced to fill out paperwork in triplicate for all eternity.
  /// You have been warned.
  AbstractValue* InitCachedValue(CacheTicket ticket,
                                 std::unique_ptr<AbstractValue> value) const {
    return cache_.Init(ticket, std::move(value));
  }

  /// Copies the given @p value into the cache entry for the given @p ticket.
  /// May throw std::bad_cast if the type of the existing value is not V.
  ///
  /// @tparam V The type of the value to store.
  template <typename V>
  void SetCachedValue(CacheTicket ticket, const V& value) const {
    cache_.Set<V>(ticket, value);
  }

  // Returns the cached value for the given @p ticket, or nullptr if the
  // cache entry has been invalidated.
  const AbstractValue* GetCachedValue(CacheTicket ticket) const {
    return cache_.Get(ticket);
  }

  // =========================================================================
  // Accessors and Mutators for Parameters.
  // TODO(david-german-tri): Add accessors for abstract parameters.

  /// Sets the parameters to @p params, deleting whatever was there before.
  void set_parameters(std::unique_ptr<Parameters<T>> params) {
    parameters_ = std::move(params);
  }

  /// Returns the entire Parameters object.
  Parameters<T>* get_mutable_parameters() {
    return parameters_.get();
  }

  /// Returns the number of vector-valued parameters.
  int num_numeric_parameters() const {
    return parameters_->num_numeric_parameters();
  }

  /// Returns a const pointer to the vector-valued parameter at @p index.
  /// Asserts if @p index doesn't exist.
  const BasicVector<T>* get_numeric_parameter(int index) const {
    return parameters_->get_numeric_parameter(index);
  }

  /// Returns a mutable pointer to element @p index of the vector-valued
  /// parameters. Asserts if @p index doesn't exist.
  BasicVector<T>* get_mutable_numeric_parameter(int index) {
    return parameters_->get_mutable_numeric_parameter(index);
  }

 protected:
  /// The caller owns the returned memory.
  Context<T>* DoClone() const override {
    LeafContext<T>* clone = new LeafContext<T>();

    // Make a deep copy of the state.
    clone->state_ = this->CloneState();

    // Make deep copies of the parameters.
    clone->set_parameters(parameters_->Clone());

    // Make deep copies of the inputs into FreestandingInputPorts.
    // TODO(david-german-tri): Preserve version numbers as well.
    for (const auto& port : this->inputs_) {
      if (port == nullptr) {
        clone->inputs_.emplace_back(nullptr);
      } else {
        clone->inputs_.emplace_back(new FreestandingInputPort(
            port->template get_vector_data<T>()->Clone()));
      }
    }

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();
    clone->cache_ = this->cache_;
    return clone;
  }

  /// The caller owns the returned memory.
  State<T>* DoCloneState() const override {
    State<T>* clone = new State<T>();

    // Make a deep copy of the continuous state using BasicVector::Clone().
    if (this->get_continuous_state() != nullptr) {
      const ContinuousState<T>& xc = *this->get_continuous_state();
      const int num_q = xc.get_generalized_position().size();
      const int num_v = xc.get_generalized_velocity().size();
      const int num_z = xc.get_misc_continuous_state().size();
      const BasicVector<T>& xc_vector =
          dynamic_cast<const BasicVector<T>&>(xc.get_vector());
      clone->set_continuous_state(std::make_unique<ContinuousState<T>>(
          xc_vector.Clone(), num_q, num_v, num_z));
    }

    // Make deep copies of the discrete and abstract states.
    clone->set_discrete_state(get_state().get_discrete_state()->Clone());
    clone->set_abstract_state(get_state().get_abstract_state()->Clone());

    return clone;
  }

  const InputPort* GetInputPort(int index) const override {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    return inputs_[index].get();
  }

 private:
  // The external inputs to the System.
  std::vector<std::unique_ptr<InputPort>> inputs_;

  // The internal state of the System.
  std::unique_ptr<State<T>> state_;

  // The parameters of the system.
  std::unique_ptr<Parameters<T>> parameters_;

  // The cache. The System may insert arbitrary key-value pairs, and configure
  // invalidation on a per-entry basis.
  mutable Cache cache_;
};

}  // namespace systems
}  // namespace drake
