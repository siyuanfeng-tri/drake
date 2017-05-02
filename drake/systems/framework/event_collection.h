#pragma once

#include <memory>
#include <vector>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/trigger.h"

namespace drake {
namespace systems {

template <typename EventType>
class NewEventCollection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NewEventCollection)

  virtual ~NewEventCollection() {}

  /**
   * Clears all the events maintained by this, and adds all the events in
   * @p other to this.
   */
  void SetFrom(const NewEventCollection<EventType>& other) {
    Clear();
    Merge(other);
  }

  /**
   * Merges all of @p other's events into this. See derived DoMerge() for more
   * details.
   */
  void Merge(const NewEventCollection<EventType>& other) {
    if (&other == this) return;
    DoMerge(&other);
  }

  /**
   * Clears all the maintained events.
   */
  virtual void Clear() = 0;

  /**
   * Returns true if no event exists.
   */
  virtual bool HasNoEvents() const = 0;

 protected:
  /**
   * Constructor only accessible by derived class.
   */
  NewEventCollection() = default;

  /**
   * Derived implementation can assume that @p is not null, and it is does not
   * equal to this.
   */
  virtual void DoMerge(const NewEventCollection<EventType>* other) = 0;
};

template <typename EventType>
class DiagramNewEventCollection final : public NewEventCollection<EventType> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramNewEventCollection)

  /**
   * Constructor. Note that this constructor only resizes the containers, but
   * does not allocate any derived NewEventCollection instances.
   *
   * @note Users should almost never call this explicitly. Use
   * System::AllocateNewEventCollection() instead.
   *
   * @param num_sub_systems Number of sub systems in the corresponding Diagram.
   */
  explicit DiagramNewEventCollection(int num_sub_systems)
      : NewEventCollection<EventType>(),
        sub_event_collection_(num_sub_systems),
        owned_sub_event_collection_(num_sub_systems) {}

  /**
   * Returns the number of constituent NewEventCollection that correspond to each
   * sub system.
   */
  int num_sub_event_collection() const {
    return static_cast<int>(sub_event_collection_.size());
  }

  /**
   * Transfers @p sub_event_collection ownership to this, and associate it with
   * sub system identified by @p index.
   */
  void set_and_own_sub_event_collection(int index,
      std::unique_ptr<NewEventCollection<EventType>> sub_event_collection);

  /**
   * Returns a const pointer to sub system's NewEventCollection at @p index.
   */
  const NewEventCollection<EventType>* get_sub_event_collection(int index) const;

  /**
   * Returns a mutable pointer to sub system's NewEventCollection at @p index.
   */
  NewEventCollection<EventType>* get_mutable_sub_event_collection(int index);

  /**
   * Goes through each sub event collection and clears its content.
   */
  void Clear() override;

  /**
   * Returns true if none of the sub event collection has any events.
   */
  bool HasNoEvents() const override;

 protected:
  // These are protected for doxygen.

  /**
   * Goes through each sub event collection and merges in the corresponding one
   * in @p other_collection. Assumes that @p other_collection is an instance of
   * DiagramNewEventCollection and has the same number of sub event collections.
   * Aborts otherwise.
   */
  void DoMerge(const NewEventCollection<EventType>* other_collection) override;

 private:
  std::vector<NewEventCollection<EventType>*> sub_event_collection_;
  std::vector<std::unique_ptr<NewEventCollection<EventType>>> owned_sub_event_collection_;
};

template <typename EventType>
class LeafNewEventCollection final : public NewEventCollection<EventType> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafNewEventCollection)

  /**
   * Constructor.
   */
  LeafNewEventCollection() = default;

  /**
   * Returns a const reference to the vector of const pointers to all the
   * publish update events.
   */
  const std::vector<const PublishEvent<T>*>& get_publish_events() const {
    return publish_events_;
  }

  /**
   * Returns a const reference to the vector of const pointers to all the
   * discrete update events.
   */
  const std::vector<const DiscreteUpdateEvent<T>*>& get_discrete_update_events()
      const {
    return discrete_update_events_;
  }

  /**
   * Returns a const reference to the vector of const pointers to all the
   * unrestricted update events.
   */
  const std::vector<const UnrestrictedUpdateEvent<T>*>&
  get_unrestricted_update_events() const {
    return unrestricted_update_events_;
  }

  /**
   * Add @p event to the existing publish event. Ownership of
   * @p event is transfered.
   */
  void add_event(std::unique_ptr<PublishEvent<T>> event) {
    owned_publish_events_.push_back(std::move(event));
    publish_events_.push_back(owned_publish_events_.back().get());
  }

  /**
   * Add @p event to the existing discrete update event. Ownership of
   * @p event is transfered.
   */
  void add_event(std::unique_ptr<DiscreteUpdateEvent<T>> event) {
    owned_discrete_update_events_.push_back(std::move(event));
    discrete_update_events_.push_back(
        owned_discrete_update_events_.back().get());
  }

  /**
   * Add @p event to the existing unrestricted update event. Ownership of
   * @p event is transfered.
   */
  void add_event(std::unique_ptr<UnrestrictedUpdateEvent<T>> event) {
    owned_unrestricted_update_events_.push_back(std::move(event));
    unrestricted_update_events_.push_back(
        owned_unrestricted_update_events_.back().get());
  }

  /**
   * Returns true if this has any publish event.
   */
  bool HasPublishEvents() const override {
    return !publish_events_.empty();
  }

  /**
   * Returns true if this has any discrete update event.
   */
  bool HasDiscreteUpdateEvents() const override {
    return !discrete_update_events_.empty();
  }

  /**
   * Returns true if this has any unrestricted update event.
   */
  bool HasUnrestrictedUpdateEvents() const override {
    return !unrestricted_update_events_.empty();
  }

  /**
   * Returns true if no event exists.
   */
  bool HasNoEvents() const override {
    return (publish_events_.empty() && discrete_update_events_.empty() &&
            unrestricted_update_events_.empty());
  }

  /**
   * Clears all events.
   */
  void Clear() override {
    owned_publish_events_.clear();
    owned_discrete_update_events_.clear();
    owned_unrestricted_update_events_.clear();
    publish_events_.clear();
    discrete_update_events_.clear();
    unrestricted_update_events_.clear();
  }

 protected:
  // These are protected for doxygen.
  void DoMerge(const NewEventCollection* other_info) override {
    const LeafNewEventCollection* other =
        dynamic_cast<const LeafNewEventCollection*>(other_info);
    DRAKE_DEMAND(other != nullptr);

    const std::vector<const PublishEvent<T>*>& other_publish =
        other->get_publish_events();
    for (const PublishEvent<T>* other_event : other_publish) {
      other_event->add_to(this);
    }

    const std::vector<const DiscreteUpdateEvent<T>*>& other_discrete_update =
        other->get_discrete_update_events();
    for (const DiscreteUpdateEvent<T>* other_event : other_discrete_update) {
      other_event->add_to(this);
    }

    const std::vector<const UnrestrictedUpdateEvent<T>*>&
        other_unrestricted_update = other->get_unrestricted_update_events();
    for (const UnrestrictedUpdateEvent<T>* other_event :
         other_unrestricted_update) {
      other_event->add_to(this);
    }
  }

 private:
  // Owned event unique pointers.
  std::vector<std::unique_ptr<PublishEvent<T>>> owned_publish_events_;
  std::vector<std::unique_ptr<DiscreteUpdateEvent<T>>>
      owned_discrete_update_events_;
  std::vector<std::unique_ptr<UnrestrictedUpdateEvent<T>>>
      owned_unrestricted_update_events_;

  // Points to the corresponding unique pointers.
  std::vector<const PublishEvent<T>*> publish_events_;
  std::vector<const DiscreteUpdateEvent<T>*> discrete_update_events_;
  std::vector<const UnrestrictedUpdateEvent<T>*> unrestricted_update_events_;
};






/**
 * Base class that represents simultaneous events at a particular time for
 * System. Each concrete event has an optional callback function for event
 * handling, and a Trigger object that holds information for the handler
 * function such as why the event occurred and optional additional data to
 * facilitate data flow from event triggering to handling.
 *
 * For each event type (publish, discrete update and unrestricted update),
 * the LeafSystem API provides a unique customizable function for handling all
 * the simultaneous events of that type, e.g.
 * LeafSystem::DoPublish(const Context&, const vector<const PublishEvent*>&)
 * for publish events, where the second argument represents all the publish
 * events that occur simultaneously for a leaf system. The default
 * implementations process the events in the same order of the second argument.
 * The user is responsible for overriding such functions to handle each event
 * in the desired order. For example, suppose two publish events are being
 * handled, events = {per step publish, periodic publish}. Depending on the
 * desired behavior, the user has the freedom to ignore both, handle only one,
 * or both in any arbitrary order. Note that for each type of events at any
 * given time, its handler should only be invoked once. The System and Diagram
 * API only provide dispatch mechanisms that delegate actual event handling
 * to all the constituient leaf systems.
 *
 * The System API provides several functions for customizable event scheduling
 * and generation such as System::DoCalcNextUpdateTime() or
 * System::DoGetPerStepEvents(). These functions can generate any number of
 * events of arbitrary types, and the resulting events are stored in separate
 * EventCollection instances. Before calling the event handlers, all these
 * EventCollection objects need to be merged, so that the handlers have a
 * complete set of simultaneous events.
 *
 * Here is a complete example. For some LeafSystem `sys` at time `t`, its
 * System::DoCalcNextUpdateTime() generates the following events (`events1`):
 * <pre>
 *   PublishEvent: {event1(kPeriodic, callback1)}
 *   DiscreteUpdateEvent: {event2(kPeriodic, callback2)}
 * </pre>
 * It also has per step events (`events2`) generated by its
 * System::DoGetPerStepEvents():
 * <pre>
 *   PublishEvent: {event3(kPerStep, callback3)}
 *   UnrestrictedUpdateEvent: {event4(kPerStep,callback4)}
 * </pre>
 * Simultaneous events `events1` and `events2` are then
 * merged into `all_events`:
 * <pre>
 *   PublishEvent: {event1, event3}
 *   DiscreteUpdateEvent: {event2}
 *   UnrestrictedUpdateEvent: {event4}
 * </pre>
 *
 * To handle these events:
 * <pre>
 *   sys.CalcUnrestrictedUpdate(context, all_events, state);
 *   sys.CalcDiscreteVariableUpdates(context, all_events, discrete_state);
 *   sys.Publish(context, all_events)
 * </pre>
 * For a LeafSystem, this is equivalent to (by expanding the dispatch mechanisms
 * in the System API):
 * <pre>
 *   sys.DoCalcUnrestrictedUpdate(context, {event4}, state);
 *   sys.DoCalcDiscreteVariableUpdates(context, {event2}, discrete_state);
 *   sys.DoPublish(context, {event1, event3})
 * </pre>
 */
class EventCollection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EventCollection)

  virtual ~EventCollection() {}

  /**
   * Clears all the events maintained by this, and adds all the events in
   * @p other to this.
   */
  void SetFrom(const EventCollection& other) {
    Clear();
    Merge(other);
  }

  /**
   * Merges all of @p other's events into this. See derived DoMerge() for more
   * details.
   */
  void Merge(const EventCollection& other) {
    if (&other == this) return;
    DoMerge(&other);
  }

  /**
   * Clears all the maintained events.
   */
  virtual void Clear() = 0;

  /**
   * Returns true if this has any publish event.
   */
  virtual bool HasPublishEvents() const = 0;

  /**
   * Returns true if this has any discrete update event.
   */
  virtual bool HasDiscreteUpdateEvents() const = 0;

  /**
   * Returns true if this has any unrestricted update event.
   */
  virtual bool HasUnrestrictedUpdateEvents() const = 0;

  /**
   * Returns true if no event exists.
   */
  virtual bool HasNoEvents() const = 0;

 protected:
  /**
   * Constructor only accessible by derived class.
   */
  EventCollection() = default;

  /**
   * Derived implementation can assume that @p is not null, and it is does not
   * equal to this.
   */
  virtual void DoMerge(const EventCollection* other) = 0;
};

/**
 * A concrete class that holds all the simultaneous events for a Diagram. For
 * each sub system in the corresponding Diagram, a derived EventCollection
 * instance is maintained internally. This effectively holds the same recursive
 * tree structure as the corresponding Diagram.
 */
class DiagramEventCollection final : public EventCollection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramEventCollection)

  /**
   * Constructor. Note that this constructor only resizes the containers, but
   * does not allocate any derived EventCollection instances.
   *
   * @note Users should almost never call this explicitly. Use
   * System::AllocateEventCollection() instead.
   *
   * @param num_sub_systems Number of sub systems in the corresponding Diagram.
   */
  explicit DiagramEventCollection(int num_sub_systems)
      : EventCollection(),
        sub_event_collection_(num_sub_systems),
        owned_sub_event_collection_(num_sub_systems) {}

  /**
   * Returns the number of constituent EventCollection that correspond to each
   * sub system.
   */
  int num_sub_event_collection() const {
    return static_cast<int>(sub_event_collection_.size());
  }

  /**
   * Transfers @p sub_event_collection ownership to this, and associate it with
   * sub system identified by @p index.
   */
  void set_and_own_sub_event_collection(int index,
      std::unique_ptr<EventCollection> sub_event_collection);

  /**
   * Returns a const pointer to sub system's EventCollection at @p index.
   */
  const EventCollection* get_sub_event_collection(int index) const;

  /**
   * Returns a mutable pointer to sub system's EventCollection at @p index.
   */
  EventCollection* get_mutable_sub_event_collection(int index);

  /**
   * Goes through each sub event collection and clears its content.
   */
  void Clear() override;

  /**
   * Returns true if this has any publish event.
   */
  bool HasPublishEvents() const override;

  /**
   * Returns true if this has any discrete update event.
   */
  bool HasDiscreteUpdateEvents() const override;

  /**
   * Returns true if this has any unrestricted update event.
   */
  bool HasUnrestrictedUpdateEvents() const override;

  /**
   * Returns true if none of the sub event collection has any events.
   */
  bool HasNoEvents() const override;

 protected:
  // These are protected for doxygen.

  /**
   * Goes through each sub event collection and merges in the corresponding one
   * in @p other_collection. Assumes that @p other_collection is an instance of
   * DiagramEventCollection and has the same number of sub event collections.
   * Aborts otherwise.
   */
  void DoMerge(const EventCollection* other_collection) override;

 private:
  std::vector<EventCollection*> sub_event_collection_;
  std::vector<std::unique_ptr<EventCollection>> owned_sub_event_collection_;
};

/**
 * A concrete class that holds all simultaneous events for a LeafSystem. For
 * each derived type of Event, all events of that type are represented by a
 * separate vector, e.g.
 * <pre>
 *   PublishEvent: {event1, event2, ...}
 *   DiscreteUpdateEvent: {event3, event4, ...}
 *   ...
 * </pre>
 */
template <typename T>
class LeafEventCollection final : public EventCollection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafEventCollection)

  /**
   * Constructor.
   */
  LeafEventCollection() = default;

  /**
   * Returns a const reference to the vector of const pointers to all the
   * publish update events.
   */
  const std::vector<const PublishEvent<T>*>& get_publish_events() const {
    return publish_events_;
  }

  /**
   * Returns a const reference to the vector of const pointers to all the
   * discrete update events.
   */
  const std::vector<const DiscreteUpdateEvent<T>*>& get_discrete_update_events()
      const {
    return discrete_update_events_;
  }

  /**
   * Returns a const reference to the vector of const pointers to all the
   * unrestricted update events.
   */
  const std::vector<const UnrestrictedUpdateEvent<T>*>&
  get_unrestricted_update_events() const {
    return unrestricted_update_events_;
  }

  /**
   * Add @p event to the existing publish event. Ownership of
   * @p event is transfered.
   */
  void add_event(std::unique_ptr<PublishEvent<T>> event) {
    owned_publish_events_.push_back(std::move(event));
    publish_events_.push_back(owned_publish_events_.back().get());
  }

  /**
   * Add @p event to the existing discrete update event. Ownership of
   * @p event is transfered.
   */
  void add_event(std::unique_ptr<DiscreteUpdateEvent<T>> event) {
    owned_discrete_update_events_.push_back(std::move(event));
    discrete_update_events_.push_back(
        owned_discrete_update_events_.back().get());
  }

  /**
   * Add @p event to the existing unrestricted update event. Ownership of
   * @p event is transfered.
   */
  void add_event(std::unique_ptr<UnrestrictedUpdateEvent<T>> event) {
    owned_unrestricted_update_events_.push_back(std::move(event));
    unrestricted_update_events_.push_back(
        owned_unrestricted_update_events_.back().get());
  }

  /**
   * Returns true if this has any publish event.
   */
  bool HasPublishEvents() const override {
    return !publish_events_.empty();
  }

  /**
   * Returns true if this has any discrete update event.
   */
  bool HasDiscreteUpdateEvents() const override {
    return !discrete_update_events_.empty();
  }

  /**
   * Returns true if this has any unrestricted update event.
   */
  bool HasUnrestrictedUpdateEvents() const override {
    return !unrestricted_update_events_.empty();
  }

  /**
   * Returns true if no event exists.
   */
  bool HasNoEvents() const override {
    return (publish_events_.empty() && discrete_update_events_.empty() &&
            unrestricted_update_events_.empty());
  }

  /**
   * Clears all events.
   */
  void Clear() override {
    owned_publish_events_.clear();
    owned_discrete_update_events_.clear();
    owned_unrestricted_update_events_.clear();
    publish_events_.clear();
    discrete_update_events_.clear();
    unrestricted_update_events_.clear();
  }

 protected:
  // These are protected for doxygen.

  /**
   * For each event type in @p other_info, adds all its events to this. Assumes
   * that @p other_info is an instance of LeafEventCollection. Aborts otherwise.
   *
   * Here is an example. Suppose this has the following events:
   * <pre>
   *   PublishEvent: {event1, event2, event3}
   *   DiscreteUpdateEvent: {event4, event5}
   * </pre>
   * @p other_info has:
   * <pre>
   *   PublishEvent: {event6}
   *   UnrestrictedUpdateEvent: {event7, event8}
   * </pre>
   * After calling DoMerge(other_info), this looks like this:
   * <pre>
   *   PublishEvent: {event1, event2, event3, event6}
   *   DiscreteUpdateEvent: {event4, event5}
   *   UnrestrictedUpdateEvent: {event7, event8}
   * </pre>
   */
  void DoMerge(const EventCollection* other_info) override {
    const LeafEventCollection* other =
        dynamic_cast<const LeafEventCollection*>(other_info);
    DRAKE_DEMAND(other != nullptr);

    const std::vector<const PublishEvent<T>*>& other_publish =
        other->get_publish_events();
    for (const PublishEvent<T>* other_event : other_publish) {
      other_event->add_to(this);
    }

    const std::vector<const DiscreteUpdateEvent<T>*>& other_discrete_update =
        other->get_discrete_update_events();
    for (const DiscreteUpdateEvent<T>* other_event : other_discrete_update) {
      other_event->add_to(this);
    }

    const std::vector<const UnrestrictedUpdateEvent<T>*>&
        other_unrestricted_update = other->get_unrestricted_update_events();
    for (const UnrestrictedUpdateEvent<T>* other_event :
         other_unrestricted_update) {
      other_event->add_to(this);
    }
  }

 private:
  // Owned event unique pointers.
  std::vector<std::unique_ptr<PublishEvent<T>>> owned_publish_events_;
  std::vector<std::unique_ptr<DiscreteUpdateEvent<T>>>
      owned_discrete_update_events_;
  std::vector<std::unique_ptr<UnrestrictedUpdateEvent<T>>>
      owned_unrestricted_update_events_;

  // Points to the corresponding unique pointers.
  std::vector<const PublishEvent<T>*> publish_events_;
  std::vector<const DiscreteUpdateEvent<T>*> discrete_update_events_;
  std::vector<const UnrestrictedUpdateEvent<T>*> unrestricted_update_events_;
};

}  // namespace systems
}  // namespace drake
