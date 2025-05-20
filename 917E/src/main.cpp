#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
 
//Defining Boolean For pneumatics
bool PneumaticPiston1 = false;
bool PneumaticPiston2 = false;

// Creating Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Setting up Drive Train Motors
pros::MotorGroup left_motors({1, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGearset::blue);
//Mechanism Motors
pros::Motor intake(7, pros::MotorGearset::blue);
pros::Motor intake2(8, pros::MotorGearset::blue);

//Pneumatics System (Has to be single quotes when calling it)
pros::adi::DigitalOut pneumaticmech('A');
pros::adi::DigitalOut pneumaticmech2('B');
// DriveTrain 
//Track Width is the distance between the center of the left wheel and center of the right wheel
//Wheel Diameter is the length of our wheels
//RPM is like the RPM of the robot
//Horizontal Drift will learn later
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
	&right_motors, // right motor group
	11.5, // 11.5 inch track width
	lemlib::Omniwheel::NEW_325, // using new 4" omnis
	450, // drivetrain rpm is 450
	2 // horizontal drift is 2 (for now)

);
//ODOMETRY CODE (reyansh will teach me about later`)
pros::Imu imu(10);


lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti wirndup
                                              1, // small eror range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// final chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
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
void competition_initialize() {}

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
	chassis.setPose(-140, 59,270);
	chassis.moveToPoint(-61, 60, 1500);
	pneumaticmech.set_value(true);
	intake.move_velocity(600);
	chassis.turnToHeading(0, 1500, {.maxSpeed = 80}, false);

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
 
 //Intake Function
void intakemechanism() {
	while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { //Outtakes thy intake
			intake.move_velocity(600);
			intake2.move_velocity(600);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { //Intake thy intake
			intake.move_velocity(-600);
			intake2.move_velocity(-600);
		} else {
			intake.move_velocity(0); // If these buttons aren't being pressed then it like stops
			intake.move_velocity(0);
		}
		pros::delay(20);
	}
}

// Pneumatics Function
void anglechangeofball() {
	while (true) {
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { 
			PneumaticPiston1 = !PneumaticPiston1; //Togles one of the pistons and puts a piece of poly carb down to change the angle of the game object
			pneumaticmech.set_value(PneumaticPiston1);//Sets Piston correct value
			PneumaticPiston2 = !PneumaticPiston2; //toggles another one of the piston that is at
			pneumaticmech2.set_value(PneumaticPiston2); //Sets Piston correct value 
		} 
		pros::delay(20);
	}
}

void opcontrol() {
	pros::Task Intake_Task (intakemechanism); // Calls Intake Funciton
	pros::Task Pneumatics_Task (anglechangeofball); //Calls Pneumatics Function
	while (true) {
		int lefty = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // Moving Forward & Backward
		int rightx = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); //Turning
		chassis.arcade(lefty, rightx); // Tells robot to move
		pros::delay(20); //Prevents freezing (very important to code)
	}
}