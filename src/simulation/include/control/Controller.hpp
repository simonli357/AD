#include "PathPlanner.hpp"
#include <memory>
#include <thread>

class Controller {
  public:
	Controller();
	Controller(Controller &&) = default;
	Controller(const Controller &) = delete;
	Controller &operator=(Controller &&) = delete;
	Controller &operator=(const Controller &) = delete;
	~Controller();

  private:
	std::thread main;
	std::unique_ptr<PathPlanner> planner;
};
