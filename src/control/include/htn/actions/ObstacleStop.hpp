#include "htn/Action.hpp"
#include <unordered_map>

class ObstacleStop : public Action {
  public:
	ObstacleStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions);
	ObstacleStop(ObstacleStop &&) = default;
	ObstacleStop(const ObstacleStop &) = default;
	ObstacleStop &operator=(ObstacleStop &&) = delete;
	ObstacleStop &operator=(const ObstacleStop &) = delete;
	~ObstacleStop();

	void execute() override;

  private:
	void update_post_conditions() override;
};
