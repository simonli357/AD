#include "htn/Action.hpp"
#include <unordered_map>

class TrafficLightStop : public Action {
  public:
	TrafficLightStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions);
	TrafficLightStop(TrafficLightStop &&) = default;
	TrafficLightStop(const TrafficLightStop &) = default;
	TrafficLightStop &operator=(TrafficLightStop &&) = delete;
	TrafficLightStop &operator=(const TrafficLightStop &) = delete;
	~TrafficLightStop();

	void execute() override;

  private:
	void update_post_conditions() override;
};
