#include "main.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "routes.h"

// Global Vars

// Robot config
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor intake(1);

pros::Imu imu_sensor(1); // Replace 1 with inertial sensor port

pros::adi::Encoder side_encoder('A', 'B', false);  // Ports 'A' and 'B' for the shaft encoder

// Program types
std::string program_type = "autonomous";
// std::string program_type = "driver"
// std::string program_type = "calibrate_metrics";

// Routes
std::vector<std::vector<double>> route = test_route; // For testing purposes
// std::vector<std::vector<double>> route = close_side;
// std::vector<std::vector<double>> route = far_side;
// std::vector<std::vector<double>> route = skills;
// std::vector<std::vector<double>> route = {}; // Driver or Calibration



double initial_heading;

// Robot parameters (needs to be tweaked later)
const double WHEEL_DIAMETER = 4.0;  // Diameter of the wheels in inches
const double TICKS_PER_ROTATION = 900.0;  // Encoder ticks per wheel rotation for green cartridges
const double WHEEL_BASE_WIDTH = 12.0;  // Distance between the left and right wheels in inches
const double DT = 0.1;  // Time step in seconds (100 ms)

// PID Code
// PID parameters (need to be tuned)
double kp_position = 1.0;
double ki_position = 0.1;
double kd_position = 0.05;

double kp_heading = 1.0;
double ki_heading = 0.1;
double kd_heading = 0.05;

// Structure to store the robot's position
struct Position {
    double x;
    double y;
    double heading;
};

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp(kp), ki(ki), kd(kd), integral(0), previous_error(0) {}

    double compute(double setpoint, double current_value, double dt) {
        double error = setpoint - current_value;
        integral += error * dt;
        double derivative = (error - previous_error) / dt;
        
        double output = kp * error + ki * integral + kd * derivative;
        previous_error = error;
        
        return output;
    }

private:
    double kp, ki, kd;
    double integral;
    double previous_error;
};


PIDController x_pid(kp_position, ki_position, kd_position);
PIDController y_pid(kp_position, ki_position, kd_position);
PIDController heading_pid(kp_heading, ki_heading, kd_heading);


// Function to convert encoder ticks to distance in inches
double ticks_to_feet(int ticks) {
    return (ticks / TICKS_PER_ROTATION) * (M_PI * WHEEL_DIAMETER) / 12;
}

// Function to get the robot's current position using encoders
Position get_robot_position(Position& current_position) {
    // Get the encoder values
	std::vector<double> left_ticks = left_mg.get_position_all();
	std::vector<double> right_ticks = right_mg.get_position_all();

	int left_tick_avg = 0;
	for (auto& tick: left_ticks){
		left_tick_avg += tick;
	}
	left_tick_avg /= left_ticks.size();

	int right_tick_avg = 0;
	for (auto& tick: right_ticks){
		right_tick_avg += tick;
	}
	right_tick_avg /= right_ticks.size();

    int side_ticks = side_encoder.get_value();

    // Calculate distances
    double left_distance = ticks_to_feet(left_tick_avg);
    double right_distance = ticks_to_feet(right_tick_avg);
    double side_distance = ticks_to_feet(side_ticks);

    // Calculate the forward and lateral displacement
    double forward_distance = (left_distance + right_distance) / 2.0;
    double lateral_distance = side_distance;

    // Get the current heading from the inertial sensor
    double current_heading = imu_sensor.get_heading();

    // Calculate the change in position
    double delta_x = forward_distance * cos(current_heading * M_PI / 180.0) - lateral_distance * sin(current_heading * M_PI / 180.0);
    double delta_y = forward_distance * sin(current_heading * M_PI / 180.0) + lateral_distance * cos(current_heading * M_PI / 180.0);

    // Update the total position
    current_position.x += delta_x;
    current_position.y += delta_y;
    current_position.heading = current_heading;

    // Reset encoders after reading
    left_mg.tare_position_all();
    right_mg.tare_position_all();
    side_encoder.reset();

    return current_position;
}

void apply_control_signal(double linear_velocity, double angular_velocity) {
    double left_speed = linear_velocity - angular_velocity;
    double right_speed = linear_velocity + angular_velocity;

    left_mg.move_velocity(left_speed);
    right_mg.move_velocity(right_speed);
}

void PID_controller(){
	double dt = DT;  // Time step in seconds (100 ms)

    static double goal_x = 0.0;
    static double goal_y = 0.0;
	static double initial_heading = route[0][1];
    Position current_position = {0.0, 0.0, 0.0};

    int index = 0;  // Index to iterate over the velocity_heading vector

    while (index < route.size()) {
        // Get the setpoints from the velocity_heading vector
        double setpoint_velocity = route[index][0];
        double setpoint_heading = route[index][1] - initial_heading;

        // Update the goal position based on setpoint_velocity and setpoint_heading
        goal_x += setpoint_velocity * DT * cos(setpoint_heading * M_PI / 180.0);
        goal_y += setpoint_velocity * DT * sin(setpoint_heading * M_PI / 180.0);

        // Get the current position and heading
        current_position = get_robot_position(current_position);

        // Compute the control signals for x, y, and heading
        double x_control_signal = x_pid.compute(goal_x, current_position.x, dt);
        double y_control_signal = y_pid.compute(goal_y, current_position.y, dt);
        double heading_control_signal = heading_pid.compute(setpoint_heading, current_position.heading, dt);

        // Combine x and y control signals to get the overall linear velocity
        double linear_velocity = sqrt(pow(x_control_signal, 2) + pow(y_control_signal, 2));

        // Apply the control signals to the motors
        apply_control_signal(linear_velocity, heading_control_signal);

        // Sleep for the time step duration
        pros::delay(dt * 1000);

        // Move to the next setpoint
        index++;
    }
}

// Structure to store velocity, acceleration, and jerk
struct MotionMetrics {
    double max_velocity;
    double max_acceleration;
    double max_jerk;
};

// Function to measure motion metrics
void measure_motion_metrics() {
    std::vector<double> velocities;
    std::vector<double> accelerations;

    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double max_jerk = 0.0;

    // Start the motors at maximum speed
    left_mg.move_velocity(200);  // Max velocity in RPM for VEX V5 motors
    left_mg.move_velocity(200);

    for (int i = 0; i < 200; ++i) {  // Run for 2 seconds (200 * 10ms)
        std::vector<double> left_ticks = left_mg.get_position_all();
		std::vector<double> right_ticks = right_mg.get_position_all();

		int left_tick_avg = 0;
		for (auto& tick: left_ticks){
			left_tick_avg += tick;
		}
		left_tick_avg /= left_ticks.size();

		int right_tick_avg = 0;
		for (auto& tick: right_ticks){
			right_tick_avg += tick;
		}
		right_tick_avg /= right_ticks.size();

        double left_distance = ticks_to_feet(left_tick_avg);
        double right_distance = ticks_to_feet(right_tick_avg);

        double velocity = (left_distance + right_distance) / 2.0 / DT;
        velocities.push_back(velocity);

        if (velocity > max_velocity) {
            max_velocity = velocity;
        }

        pros::delay(DT * 1000);  // Delay for DT seconds

        left_mg.tare_position_all();
        right_mg.tare_position_all();
    }

    // Stop the motors
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);

    // Calculate accelerations
    for (size_t i = 1; i < velocities.size(); ++i) {
        double acceleration = (velocities[i] - velocities[i - 1]) / DT;
        accelerations.push_back(acceleration);

        if (acceleration > max_acceleration) {
            max_acceleration = acceleration;
        }
    }

    // Calculate jerk
    for (size_t i = 1; i < accelerations.size(); ++i) {
        double jerk = (accelerations[i] - accelerations[i - 1]) / DT;

        if (jerk > max_jerk) {
            max_jerk = jerk;
        }
    }

	pros::lcd::initialize();
    pros::lcd::print(0, "Max Velocity: %.2f ft/s", max_velocity);
    pros::lcd::print(1, "Max Acceleration: %.2f ft/s^2", max_acceleration);
    pros::lcd::print(2, "Max Jerk: %.2f ft/s^3", max_jerk);
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);

	left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
	right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

	// Calibrate the inertial sensor
    imu_sensor.reset();

  	imu_sensor.reset();
	initial_heading = imu_sensor.get_heading();

	// autonomous() // For outside of competition testing purposes
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	if (program_type == "calibrate_metrics"){
		measure_motion_metrics();
	} else {
    	PID_controller();
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}