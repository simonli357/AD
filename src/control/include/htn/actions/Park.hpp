#include "htn/Action.hpp"
#include <unordered_map>

class Park : public Action {
  public:
	Park(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions);
	Park(Park &&) = default;
	Park(const Park &) = default;
	Park &operator=(Park &&) = delete;
	Park &operator=(const Park &) = delete;
	~Park();

	void execute() override;

  private:
	void update_post_conditions() override;
};
