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
		/* if (car_started && idx == 0) { */
		/*     break; */
		/* } */
		const Vertex &v = path[idx];
		/* if (!start_car || path.empty()) { */
		/* check_if_can_start(v.x, v.y, idx); */
		/* 	rate.sleep(); */
		/* 	continue; */
		/* } */
		car_started = true;
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
	if (path.empty())
		return;
	for (size_t step = 1; step <= lookahead_wpts; ++step) {
		size_t i = (idx + path.size() - step) % path.size();
		const auto &wp = path[i];

		const auto pred = [this, &wp](double cx, double cy) { return is_near(wp.x, wp.y, cx, cy, wp_radius, car_radius); };

		if (traffic_manager.car_in_front("car1", pred)) {
			start_car = true;
		}
	}
}

void HighwayCar::get_path() {
	path.clear();
	planner->set_constraints(vref, N, T, car_path);
	planner->print_path();
	path = planner->plan_path();
}
