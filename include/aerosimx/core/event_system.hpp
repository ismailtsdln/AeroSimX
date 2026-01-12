#pragma once

/**
 * @file event_system.hpp
 * @brief Event-driven communication system
 */

#include <any>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include "aerosimx/core/types.hpp"

namespace aerosimx {
namespace core {

/**
 * @brief Event priority levels
 */
enum class EventPriority { Low = 0, Normal = 1, High = 2, Critical = 3 };

/**
 * @brief Base event structure
 */
struct Event {
  std::string type;
  Timestamp timestamp;
  EntityId source_id = INVALID_ENTITY_ID;
  EventPriority priority = EventPriority::Normal;

  virtual ~Event() = default;
};

/**
 * @brief Collision event
 */
struct CollisionEvent : public Event {
  EntityId entity_a;
  EntityId entity_b;
  Vector3 contact_point;
  Vector3 contact_normal;
  double impact_velocity;
};

/**
 * @brief Vehicle state change event
 */
struct VehicleStateEvent : public Event {
  EntityId vehicle_id;
  VehicleState new_state;
  VehicleState previous_state;
};

/**
 * @brief Sensor data event
 */
struct SensorDataEvent : public Event {
  EntityId sensor_id;
  std::string sensor_type;
  std::any data; // Type-erased sensor data
};

/**
 * @brief Type-safe event handler function
 */
template <typename EventType>
using EventHandler = std::function<void(const EventType &)>;

/**
 * @brief Subscription ID for event unsubscription
 */
using SubscriptionId = uint64_t;

/**
 * @brief Event-driven communication system
 *
 * Provides a publish-subscribe mechanism for decoupled
 * communication between simulation components.
 */
class EventSystem {
public:
  EventSystem();
  ~EventSystem();

  /**
   * @brief Subscribe to events of a specific type
   * @tparam EventType The event type to subscribe to
   * @param handler The handler function
   * @param priority Handler priority (higher = called first)
   * @return Subscription ID for unsubscription
   */
  template <typename EventType>
  SubscriptionId subscribe(EventHandler<EventType> handler,
                           EventPriority priority = EventPriority::Normal);

  /**
   * @brief Subscribe to events by type name
   * @param event_type Event type name
   * @param handler Generic handler
   * @return Subscription ID
   */
  SubscriptionId subscribe(const std::string &event_type,
                           std::function<void(const Event &)> handler,
                           EventPriority priority = EventPriority::Normal);

  /**
   * @brief Unsubscribe from events
   * @param subscription_id The subscription ID returned from subscribe
   */
  void unsubscribe(SubscriptionId subscription_id);

  /**
   * @brief Publish an event
   * @tparam EventType The event type
   * @param event The event to publish
   */
  template <typename EventType> void publish(const EventType &event);

  /**
   * @brief Publish an event by base class
   */
  void publish(const Event &event);

  /**
   * @brief Queue an event for later processing
   */
  template <typename EventType> void queue(const EventType &event);

  /**
   * @brief Process all queued events
   */
  void process_queue();

  /**
   * @brief Clear the event queue
   */
  void clear_queue();

  /**
   * @brief Get number of subscriptions
   */
  size_t subscription_count() const;

  /**
   * @brief Get number of queued events
   */
  size_t queue_size() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Template implementation
template <typename EventType>
SubscriptionId EventSystem::subscribe(EventHandler<EventType> handler,
                                      EventPriority priority) {
  auto wrapper = [handler](const Event &base_event) {
    const auto *typed_event = dynamic_cast<const EventType *>(&base_event);
    if (typed_event) {
      handler(*typed_event);
    }
  };
  return subscribe(typeid(EventType).name(), wrapper, priority);
}

template <typename EventType>
void EventSystem::publish(const EventType &event) {
  publish(static_cast<const Event &>(event));
}

template <typename EventType> void EventSystem::queue(const EventType &event) {
  // Implementation handled in cpp file
}

} // namespace core
} // namespace aerosimx
