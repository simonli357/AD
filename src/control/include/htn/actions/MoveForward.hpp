#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class MoveForward : public Action {
  public:
	MoveForward(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	MoveForward(MoveForward &&) = delete;
	MoveForward(const MoveForward &) = delete;
	MoveForward &operator=(MoveForward &&) = delete;
	MoveForward &operator=(const MoveForward &) = delete;
	~MoveForward();

	void execute() override;

  private:
	void update_post_conditions() override;
	void solve();
	bool detect_stop_sign();
	bool detect_traffic_light();
	bool detect_parking_sign();
	bool detect_obstacles();
	bool destination_reached();
};
