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
	void solve();
	bool destination_reached();
};
