#include "drake/systems/primitives/selector.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template <typename T>
Selector<T>::Selector(int input_size, const std::vector<int>& input_indices) {
  this->DeclareInputPort(kVectorValued, input_size);
  this->DeclareOutputPort(kVectorValued, input_indices.size());

  for (int i = 0; i < static_cast<int>(input_indices.size()); i++) {
    DRAKE_DEMAND(input_indices[i] < input_size);
    input_to_output_.emplace(input_indices[i], i);
    output_to_input_.emplace(i, input_indices[i]);
  }
}

template <typename T>
void Selector<T>::DoCalcOutput(const Context<T>& context,
                                    SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto in_vector = System<T>::EvalEigenVectorInput(context, 0);
  auto out_vector = System<T>::GetMutableOutputVector(output, 0);
  for (const auto& pair : input_to_output_) {
    out_vector[pair.second] = in_vector[pair.first];
  }
}

template class Selector<double>;
template class Selector<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
