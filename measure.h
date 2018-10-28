
#include <string>



class TimeMeasurement {
 public:
  enum class EventType {None, Begin, End};
 private:
  struct event_entry {
    std::string& name;
    EventType event;
  };

  static size_t first_empty = 0;
  static const size_t kEventEntriesCount = 1000;
  static thread_local event_entry events[kEventEntriesCount];
  static std::string empty_name;

 public:
  static void Init() {
    first_empty = 0;
    event_entry empty = {
      .name = empty_name, 
      .event = EventType::None
      };
    std::fill(events, events + kEventEntriesCount, empty);
  }

  static void Event(std::string& name, EventType event) {
    event_entry this_event = {
      .name = name,
      .event = event
    };

    events[first_empty++] = this_event;
  }

 private: 

}