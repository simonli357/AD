#include "htn/Action.hpp"
#include <unordered_map>

class Move : public Action {
  public:
	Move(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions);
	Move(Move &&) = default;
	Move(const Move &) = default;
	Move &operator=(Move &&) = delete;
	Move &operator=(const Move &) = delete;
	~Move();

	void execute() override;

  private:
	void update_post_conditions() override;
	bool detect_stop_sign();
	bool detect_traffic_light();
	bool detect_parking_sign();
	bool detect_obstacles();
	bool destination_reached();
};
