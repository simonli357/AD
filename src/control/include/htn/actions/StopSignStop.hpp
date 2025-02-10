#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class StopSignStop : public Action {
  public:
	StopSignStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions);
	StopSignStop(StopSignStop &&) = delete;
	StopSignStop(const StopSignStop &) = delete;
	StopSignStop &operator=(StopSignStop &&) = delete;
	StopSignStop &operator=(const StopSignStop &) = delete;
	~StopSignStop();

	void execute() override;

  private:
	void update_post_conditions() override;
};
