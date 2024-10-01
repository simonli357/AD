#include "optimizer.hpp"

int main() {
    Optimizer controller;
    controller.x_current << controller.state_refs(0, 0), controller.state_refs(0, 1), controller.state_refs(0, 2);
    // controller.change_lane(223, 238);
    // controller.change_lane(123, 143);
    // controller.change_lane(15, 30);
    // controller.change_lane(280, 295);
    // controller.change_lane(410, 430);
    // controller.change_lane(485, 500);
    controller.run();
    // controller.x_current << 0, 0, 0;
    // controller.park();
    controller.computeStats(0);
    return 0;
}
