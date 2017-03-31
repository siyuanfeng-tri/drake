#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <iostream>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcmInterface;

using std::make_unique;

// TODO(jwnimmer-tri) The "serializer xor translator" disjoint implementations
// within the method bodies below are not ideal, because of the code smell, and
// because it is likely confusing for users.  We should take further steps to
// make the Value<LcmMessage> port the primary output port, and find a better
// phrasing for the vector-valued output port for users.  For now though, this
// implementation serves as a transition point where we don't have to rewrite
// the old code yet, but still can supply the AbstractValue port for new code.

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator* translator,
    std::unique_ptr<SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm)
    : channel_(channel),
      translator_(translator),
      serializer_(std::move(serializer)) {
  DRAKE_DEMAND((translator_ != nullptr) != (serializer_.get() != nullptr));
  DRAKE_DEMAND(lcm);

  lcm->Subscribe(channel_, this);
  if (translator_ != nullptr) {
    DeclareOutputPort(kVectorValued, translator_->get_vector_size());
  } else {
    DeclareAbstractOutputPort();
  }

  set_name(make_name(channel_));
}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, nullptr, std::move(serializer), lcm) {}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator& translator, DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, &translator, nullptr, lcm) {}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary,
    DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, translator_dictionary.GetTranslator(channel),
                          lcm) {}

LcmSubscriberSystem::~LcmSubscriberSystem() {}

void LcmSubscriberSystem::DoCalcNextUpdateTime(
    const Context<double>& context, UpdateActions<double>* events) const {
  std::unique_lock<std::mutex> lock(received_message_mutex_);
  const int last_message_count = context.get_abstract_state<int>(1);
  // Has a new message. Schedule an unrestricted update event.
  if (last_message_count != received_message_count_) {
    events->time = context.get_time() + 0.0001;
    DiscreteEvent<double> event;
    event.action = DiscreteEvent<double>::ActionType::kUnrestrictedUpdateAction;
    events->events.push_back(event);
  } else {
    events->time = std::numeric_limits<double>::infinity();
  }
}

void LcmSubscriberSystem::DoCalcUnrestrictedUpdate(
    const Context<double>& context, State<double>* state) const {
  AbstractValue& msg = state->get_mutable_abstract_state()->get_mutable_value(0);
  AbstractValue& msg_ctr = state->get_mutable_abstract_state()->get_mutable_value(1);

  if (translator_ != nullptr) {
    VectorBase<double>* const output_vector = &msg.GetMutableValue<BasicVector<double>>();
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    if (!received_message_.empty()) {
      translator_->Deserialize(
          received_message_.data(), received_message_.size(), output_vector);
      msg_ctr.GetMutableValue<int>() = received_message_count_;
    }
  } else {
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    if (!received_message_.empty()) {
      serializer_->Deserialize(
          received_message_.data(), received_message_.size(), &msg);
      msg_ctr.GetMutableValue<int>() = received_message_count_;
    }

  }
}

std::unique_ptr<AbstractValues> LcmSubscriberSystem::AllocateAbstractState() const {
  // Index 0 is the message, 1 is the message counter.
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
  // VectorBase output.
  if (translator_ != nullptr) {
    DRAKE_DEMAND(serializer_ == nullptr);
    auto v = std::make_unique<BasicVector<double>>(translator_->get_vector_size());
    abstract_vals[0] = std::make_unique<VectorValue<double>>(std::move(v));
  } else {
    DRAKE_DEMAND(serializer_ != nullptr);
    abstract_vals[0] = serializer_->CreateDefaultValue();
  }
  abstract_vals[1] = AbstractValue::Make<int>(0);
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

std::string LcmSubscriberSystem::make_name(const std::string& channel) {
  return "LcmSubscriberSystem(" + channel + ")";
}

const std::string& LcmSubscriberSystem::get_channel_name() const {
  return channel_;
}

void LcmSubscriberSystem::DoCalcOutput(const Context<double>& context,
                                       SystemOutput<double>* output) const {
  DRAKE_ASSERT((translator_ != nullptr) != (serializer_.get() != nullptr));

  if (translator_ != nullptr) {
    VectorBase<double>* const output_vector = output->GetMutableVectorData(0);
    DRAKE_ASSERT(output_vector != nullptr);

    const BasicVector<double>& msg = context.get_abstract_state<BasicVector<double>>(0);
    output_vector->SetFrom(msg);
  } else {
    AbstractValue* const output_value = output->GetMutableData(0);
    DRAKE_ASSERT(output_value != nullptr);

    output_value->SetFrom(context.get_abstract_state()->get_value(0));
  }
}

// This is only called if our output port is vector-valued.
std::unique_ptr<BasicVector<double>> LcmSubscriberSystem::AllocateOutputVector(
    const OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == 0);
  DRAKE_DEMAND(descriptor.get_data_type() == kVectorValued);
  DRAKE_DEMAND(translator_ != nullptr);
  DRAKE_DEMAND(serializer_ == nullptr);
  auto result = translator_->AllocateOutputVector();
  if (result) {
    return result;
  }
  return LeafSystem<double>::AllocateOutputVector(descriptor);
}

// This is only called if our output port is abstract-valued.
std::unique_ptr<systems::AbstractValue>
LcmSubscriberSystem::AllocateOutputAbstract(
    const OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == 0);
  DRAKE_DEMAND(descriptor.get_data_type() == kAbstractValued);
  DRAKE_DEMAND(translator_ == nullptr);
  DRAKE_DEMAND(serializer_ != nullptr);
  return serializer_->CreateDefaultValue();
}

void LcmSubscriberSystem::HandleMessage(const std::string& channel,
    const void* message_buffer, int message_size) {
  SPDLOG_TRACE(drake::log(), "Receiving LCM {} message", channel);

  if (channel == channel_) {
    const uint8_t* const rbuf_begin =
        reinterpret_cast<const uint8_t*>(message_buffer);
    const uint8_t* const rbuf_end = rbuf_begin + message_size;
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    received_message_.clear();
    received_message_.insert(received_message_.begin(), rbuf_begin, rbuf_end);
    received_message_count_++;
    received_message_condition_variable_.notify_all();
  } else {
    std::cerr << "LcmSubscriberSystem: HandleMessage: WARNING: Received a "
              << "message for channel \"" << channel
              << "\" instead of channel \"" << channel_ << "\". Ignoring it."
              << std::endl;
  }
}

int LcmSubscriberSystem::WaitForMessage(int old_message_count) const {
  std::unique_lock<std::mutex> lock(received_message_mutex_);
  while (old_message_count == received_message_count_)
    received_message_condition_variable_.wait(lock);
  int new_message_count = received_message_count_;
  lock.unlock();

  return new_message_count;
}

const LcmAndVectorBaseTranslator& LcmSubscriberSystem::get_translator() const {
  DRAKE_DEMAND(translator_ != nullptr);
  return *translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
