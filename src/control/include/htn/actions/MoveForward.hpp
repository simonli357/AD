#include "htn/Action.hpp"
#include <unordered_map>

class MoveForward : public Action {
  public:
	MoveForward(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions);
	MoveForward(MoveForward &&) = default;
	MoveForward(const MoveForward &) = default;
	MoveForward &operator=(MoveForward &&) = delete;
	MoveForward &operator=(const MoveForward &) = delete;
	~MoveForward();

	void execute() override;

  private:
	void update_post_conditions() override;
	bool detect_stop_sign();
	bool detect_traffic_light();
	bool detect_parking_sign();
	bool detect_obstacles();
	bool destination_reached();
};
