// KeyboardController.hpp
#ifndef KEYBOARD_CONTROLLER_HPP
#define KEYBOARD_CONTROLLER_HPP

#include <ncurses.h>

class KeyboardController {
public:
    KeyboardController() {
        // Initialize ncurses mode
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        timeout(100);

        printw("Use 'w' and 's' to increase or decrease speed.\n");
        printw("Use 'a' and 'd' to control steering.\n");
        printw("'q' to quit.\n");

        running = true;

    }

    bool running = true;
    int runOnce(double oSteering, double oVelocity) {
        int ch = getch();
        ch = getch();
                
        switch (ch) {
            case 'w':
                velocity += VELOCITY_INCREMENT;
                if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
                return 1;
            case 's':
                velocity -= VELOCITY_INCREMENT;
                if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
                return 1;
            case 'a':
                steering_angle -= STEERING_INCREMENT;
                if (steering_angle < -HARD_MAX_STEERING) steering_angle = -HARD_MAX_STEERING;
                return 1;
            case 'd':
                steering_angle += STEERING_INCREMENT;
                if (steering_angle > HARD_MAX_STEERING) steering_angle = HARD_MAX_STEERING;
                return 1;
            case 'b':
                velocity = 0;
                return 1;
            case 'q':
                running = false;
                return 0;
            default:
                // Gradually return the steering towards zero if no steering keys are pressed
                if (steering_angle > 0) {
                    steering_angle -= STEERING_DECAY;
                    if (steering_angle < 0) steering_angle = 0;
                } else if (steering_angle < 0) {
                    steering_angle += STEERING_DECAY;
                    if (steering_angle > 0) steering_angle = 0;
                }
                if (velocity > 0) {
                    velocity -= VELOCITY_DECAY;
                    if (velocity < 0) velocity = 0;
                } else if (velocity < 0) {
                    velocity += VELOCITY_DECAY;
                    if (velocity > 0) velocity = 0;
                }
                return 1;
        }

        // Clear previous outputs
        // clear();
        printw("Velocity: %f\n", velocity);
        printw("Steering angle: %f\n", steering_angle);
        printw("Press 'q' to quit.");
        
        oSteering = steering_angle;
        oVelocity = velocity;
        return 1;
    }
    void clean() {
        endwin();
    }
private:
    // Constants for steering and speed
    const double STEERING_INCREMENT = 2;
    const double VELOCITY_INCREMENT = 0.05;
    const double MAX_VELOCITY = 0.45;
    const double MIN_VELOCITY = -0.45;
    const double HARD_MAX_STEERING = 25.0;
    const double STEERING_DECAY = 1.25;
    const double VELOCITY_DECAY = 0;  // 0.0025;

    double velocity = 0.0;
    double steering_angle = 0.0;
};

#endif // KEYBOARD_CONTROLLER_HPP
