/**
 * @file event_system.cpp
 * @brief Event-driven communication system implementation
 */

#include "aerosimx/core/event_system.hpp"

#include <algorithm>
#include <queue>

namespace aerosimx {
namespace core {

struct Subscription {
  SubscriptionId id;
  std::string event_type;
  std::function<void(const Event &)> handler;
  EventPriority priority;
};

struct EventSystem::Impl {
  std::vector<Subscription> subscriptions;
  std::queue<std::unique_ptr<Event>> event_queue;
  std::mutex subscriptions_mutex;
  std::mutex queue_mutex;
  std::atomic<SubscriptionId> next_subscription_id{1};
};

EventSystem::EventSystem() : impl_(std::make_unique<Impl>()) {}

EventSystem::~EventSystem() = default;

SubscriptionId
EventSystem::subscribe(const std::string &event_type,
                       std::function<void(const Event &)> handler,
                       EventPriority priority) {
  std::lock_guard<std::mutex> lock(impl_->subscriptions_mutex);

  SubscriptionId id = impl_->next_subscription_id++;

  Subscription sub;
  sub.id = id;
  sub.event_type = event_type;
  sub.handler = std::move(handler);
  sub.priority = priority;

  impl_->subscriptions.push_back(std::move(sub));

  // Sort by priority (higher priority first)
  std::sort(impl_->subscriptions.begin(), impl_->subscriptions.end(),
            [](const Subscription &a, const Subscription &b) {
              return static_cast<int>(a.priority) >
                     static_cast<int>(b.priority);
            });

  return id;
}

void EventSystem::unsubscribe(SubscriptionId subscription_id) {
  std::lock_guard<std::mutex> lock(impl_->subscriptions_mutex);

  impl_->subscriptions.erase(
      std::remove_if(impl_->subscriptions.begin(), impl_->subscriptions.end(),
                     [subscription_id](const Subscription &sub) {
                       return sub.id == subscription_id;
                     }),
      impl_->subscriptions.end());
}

void EventSystem::publish(const Event &event) {
  std::lock_guard<std::mutex> lock(impl_->subscriptions_mutex);

  for (const auto &sub : impl_->subscriptions) {
    if (sub.event_type == event.type ||
        sub.event_type == typeid(event).name()) {
      try {
        sub.handler(event);
      } catch (...) {
        // Swallow exceptions from handlers
      }
    }
  }
}

void EventSystem::process_queue() {
  std::queue<std::unique_ptr<Event>> events_to_process;

  {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    std::swap(events_to_process, impl_->event_queue);
  }

  while (!events_to_process.empty()) {
    auto &event = events_to_process.front();
    if (event) {
      publish(*event);
    }
    events_to_process.pop();
  }
}

void EventSystem::clear_queue() {
  std::lock_guard<std::mutex> lock(impl_->queue_mutex);
  std::queue<std::unique_ptr<Event>> empty;
  std::swap(impl_->event_queue, empty);
}

size_t EventSystem::subscription_count() const {
  std::lock_guard<std::mutex> lock(impl_->subscriptions_mutex);
  return impl_->subscriptions.size();
}

size_t EventSystem::queue_size() const {
  std::lock_guard<std::mutex> lock(impl_->queue_mutex);
  return impl_->event_queue.size();
}

} // namespace core
} // namespace aerosimx
