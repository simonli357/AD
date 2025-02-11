#include "htn/actions/MoveForward.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

MoveForward::MoveForward(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
    cost = 2; // Moving forward is the best move as it moves us closer to the goal.
	pre_conditions = {
        {FORCE_STOP, false},
        {TRAFFIC_LIGHT_DETECTED, false},
        {STOP_SIGN_DETECTED, false},
        {PEDESTRIAN_DETECTED, false},
        {DESTINATION_REACHED, false},
        {CAR_DETECTED_ON_SAME_LANE, false},
	};
}

MoveForward::~MoveForward() {}

void MoveForward::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    // Move the car forward
    utils.debug("Performing Action: Move Forward.", 2);
	world.update_mpc_states();
	solve();
    current_state[DESTINATION_REACHED] = destination_reached();
    world.rate->sleep();
}

void MoveForward::solve() {
	int success = path_manager.find_next_waypoint(path_manager.target_waypoint_index, x_current);
	// std::cout << "current state: x: " << x_current(0) << ", y: " << x_current(1) << ", yaw: " << x_current(2) << std::endl;
	// std::cout << "closest waypoint index: " << path_manager.closest_waypoint_index << ", at x: " << path_manager.state_refs(path_manager.closest_waypoint_index, 0) << ", y: " <<
	// path_manager.state_refs(path_manager.closest_waypoint_index, 1) << ", yaw: " << path_manager.state_refs(path_manager.closest_waypoint_index, 2) << std::endl; std::cout <<
	// "target waypoint index: " << path_manager.target_waypoint_index << ", at x: " << path_manager.state_refs(path_manager.target_waypoint_index, 0) << ", y: " <<
	// path_manager.state_refs(path_manager.target_waypoint_index, 1) << ", yaw: " << path_manager.state_refs(path_manager.target_waypoint_index, 2) << std::endl; for (int i =
	// path_manager.target_waypoint_index; i < std::min(path_manager.target_waypoint_index + 6, static_cast<int>(path_manager.state_refs.rows())); i++) {
	//     std::cout << "i: " << i << ", x: " << path_manager.state_refs(i, 0) << ", y: " << path_manager.state_refs(i, 1) << ", yaw: " << path_manager.state_refs(i, 2) <<
	//     std::endl;
	// }
	int idx = path_manager.target_waypoint_index;
	if (idx > path_manager.state_refs.rows() - 2) {
		idx = path_manager.state_refs.rows() - 2;
		utils.debug("WARNING: solve(): target waypoint index exceeds state_refs size, using last waypoint...", 3);
	}
	int N = std::min(Eigen::Index(path_manager.N), path_manager.state_refs.rows() - idx);
	if (idx >= 0 && idx <= path_manager.state_refs.rows() - 2 && N > 0) {

		Eigen::Block<Eigen::MatrixXd> state_refs_block = path_manager.state_refs.block(idx, 0, N, 3);
		Eigen::Block<Eigen::MatrixXd> input_refs_block = path_manager.input_refs.block(idx, 0, N, 2);
		int status = world.mpc.solve(state_refs_block, input_refs_block, x_current);
	} else {
		ROS_WARN("Block indices are out of bounds, skipping solve.");
		ROS_INFO("state_refs rows: %ld, cols: %ld", path_manager.state_refs.rows(), path_manager.state_refs.cols());
		ROS_INFO("input_refs rows: %ld, cols: %ld", path_manager.input_refs.rows(), path_manager.input_refs.cols());
		ROS_INFO("idx: %d, N: %d", idx, N);
	}
	world.publish_commands();
}

bool MoveForward::destination_reached() {
	double error_sq = (x_current.head(2) - world.destination).squaredNorm();
	if (error_sq < TOLERANCE_SQUARED && path_manager.target_waypoint_index >= path_manager.state_refs.rows() * 0.9) {
		return true;
	}
	return false;
}
