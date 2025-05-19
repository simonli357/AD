#include "HighwayCar.hpp"
#include <string>

HighwayCar::HighwayCar(void *traffic_manager, ros::NodeHandle &nh, double vref, const std::string &car_name, const std::string &path_name) : Car(traffic_manager, nh, vref, car_name) { this->car_path = path_name; }

void HighwayCar::start() {
	get_path();
	Vertex start = path[0];
	std::cout << "HIGHWAY CAR: " << car_name << " initialized." << std::endl;
	traffic_manager->task_manager->run([this] { follow_path(); });
}

void HighwayCar::follow_path() {
	ros::Rate rate(1.0 / T);
	size_t idx = 0;
	bool stopped = true;
	while (ros::ok() && alive) {
		const Vertex &v = path[idx];
		if (!start_car || path.empty()) {
			move_car_to(v.x, v.y, v.tangent_angle);
            check_if_can_start(v.x, v.y, idx);
			rate.sleep();
			continue;
		}
		// Collision detection. If there is an obstacle in front of us, do not move
		if (can_move_car(v.x, v.y, idx)) {
			move_car_to(v.x, v.y, v.tangent_angle);
		} else {
			rate.sleep();
			continue;
		}
		// If we are at a stopline, stop for 3 seconds
		if (v.attribute == ATTR::STOPLINE && !stopped) {
			ros::Duration(3.0).sleep();
			stopped = true;
		}
		if (v.attribute != ATTR::STOPLINE && stopped) {
			stopped = false;
		}
		idx = (idx + 1) % path.size();
		rate.sleep();
	}
}

void HighwayCar::check_if_can_start(double car_x, double car_y, size_t idx) {
	if (path.empty()) {
		return;
    }

    double ego_x, ego_y;
    traffic_manager->get_car_position("car1", ego_x, ego_y);

    double self_x, self_y;
    traffic_manager->get_car_position(car_name, self_x, self_y);

    double r1 = 0.5;
    double r2 = 0.5;

    if (is_near(ego_x, ego_y, self_x, self_y, r1, r2)) {
        start_car = true;
    }
}

void HighwayCar::get_path() {
	path.clear();
	planner->set_constraints(vref, N, T, car_path);
	planner->print_path();
	path = planner->plan_path();
}
