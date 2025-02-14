#include "htn/actions/TrafficOvertake.hpp"
#include "World.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

TrafficOvertake::TrafficOvertake(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
	cost = 1;
	pre_conditions = {
		{FORCE_STOP, false},
		{CAR_DETECTED_ON_SAME_LANE, true},
        {LANE_IS_DOTTED, true}
	};
}

TrafficOvertake::~TrafficOvertake() {}

void TrafficOvertake::execute() {
	if (!can_execute()) {
		utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
	// Stop before tailing the car
	utils.debug("Performing Action: Traffic Overtake.", 2);
    
    double x, y, yaw;
    utils.get_states(x, y, yaw);
    double dist;
    bool right = false;
    double start_dist = std::max(dist - CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
    double density = path_manager.density;
    int closest_idx = path_manager.find_closest_waypoint(x_current);
    int start_index = closest_idx + static_cast<int>(start_dist * density);
    double min_dist_sq = 1000.;
    double min_dist_sq_adj = 1000.; // distance to adjacent lane
    double min_dist = std::sqrt(min_dist_sq);
    double min_dist_adj = std::sqrt(min_dist_sq_adj);
    static double lane_offset = LANE_OFFSET * world.change_lane_offset_scaler ;

    if (start_index >= path_manager.state_refs.rows() || path_manager.overtake_end_index >= path_manager.state_refs.rows()) {
        utils.debug("CHECK_CAR(): WARNING: start or end index exceeds state_refs size, stopping...", 2);
        current_state[CAR_DETECTED_ON_SAME_LANE] = false;
        current_state[LANE_IS_DOTTED] = false;
        return;
    };

    path_manager.overtake_end_index = start_index + static_cast<int>((CAR_LENGTH * 2 + MIN_DIST_TO_CAR * 2) * density * path_manager.overtake_end_index_scaler);
    utils.debug("CHECK_CAR(): SAME_LANE: OVERTAKING: start idx: " + std::to_string(start_index) + ", end idx: " + std::to_string(path_manager.overtake_end_index) + ", min_dist: " + std::to_string(min_dist) + ", min_dist_adj: " + std::to_string(min_dist_adj), 2);
    path_manager.change_lane(start_index, path_manager.overtake_end_index, right, lane_offset);
    utils.debug("CHECK_CAR(): SAME_LANE: OVERTAKING: changing lane to the " + std::string(right ? "right" : "left") + " in " + std::to_string(start_dist) + " meters. start pose: (" + std::to_string(path_manager.state_refs(start_index, 0)) + "," + std::to_string(path_manager.state_refs(start_index, 1)) + "), end: (" + std::to_string(path_manager.state_refs(path_manager.overtake_end_index, 0)) + ", " + std::to_string(path_manager.state_refs(path_manager.overtake_end_index, 1)) + "), cur: (" + std::to_string(x) + ", " + std::to_string(y) + ")", 2);

    current_state[CAR_DETECTED_ON_SAME_LANE] = false;
    current_state[LANE_IS_DOTTED] = false;
}
