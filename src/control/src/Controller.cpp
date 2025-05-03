#include <Controller.hpp>

StateMachine *globalStateMachinePtr = nullptr;
void signalHandler(int signum) {
    if (globalStateMachinePtr) {
        globalStateMachinePtr->utils.stop_car();
        globalStateMachinePtr->call_trigger_service();
    }
    if (globalStateMachinePtr->utils.serial && globalStateMachinePtr->utils.serial->is_open()) {
        globalStateMachinePtr->utils.serial->close();
    }
    globalStateMachinePtr->utils.serial.reset();
    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv) {
    std::cout.precision(3);
    ros::init(argc, argv, "mpc_node", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    Database db;
    VehicleConstants::init_params(db);

    if (!Tunable::loadFromParams(nh)) {
        std::cout << "FATAL ERROR: Failed to load tunable parameters" << std::endl;
        exit(1);
    }
    GroundTruth::initialize_ground_truth();
    Tracking::initialize_tracking();
    StateMachine sm(nh, db);

    globalStateMachinePtr = &sm;
    signal(SIGINT, signalHandler);
    
    std::thread callback_thread;
    std::unique_ptr<ros::AsyncSpinner> spinner;
    if (async) {
        int num_threads = 4;
        spinner = std::make_unique<ros::AsyncSpinner>(num_threads);
        spinner->start();
        std::cout << "Async spinner started with " << num_threads << " threads" << std::endl;
    } else {
        callback_thread = std::thread(&Utility::spin, &sm.utils);
    }
    
    sm.run();

    ros::waitForShutdown();
    if (callback_thread.joinable()) {
        callback_thread.join();
    }
    return 0;
}
