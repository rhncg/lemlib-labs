#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

pros::MotorGroup leftMotors({1,2,3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({4,5,6}, pros::MotorGearset::blue);

pros::Rotation vertical(7);
pros::Rotation horizontal(8);

pros::Imu imu(9);

pros::Controller Master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain drivetrain(
	&leftMotors,
	&rightMotors,
	11,
	lemlib::Omniwheel::NEW_325,
	450,
	2
);

lemlib::ControllerSettings lateralPID(
    5,  // proportional gain (kP)
    0,  // integral gain (kI)
    4, // derivative gain (kD)
    0, // anti windup
    0, // small error range, in inches 1
    0, // small error range timeout, in milliseconds 100
    0, // large error range, in inches 3
    0, // large error range timeout, in milliseconds 500
    0 // maximum acceleration (slew) 20
);

lemlib::ControllerSettings angularPID(
    3,  // proportional gain (kP)
    0,  // integral gain (kI)
    24, // derivative gain (kD)
    0, // anti windup
    0.1, // small error range, in inches 1
    1000, // small error range timeout, in milliseconds 100
    0, // large error range, in inches 3
    0, // large error range timeout, in milliseconds 500
    0 // maximum acceleration (slew) 20
);

lemlib::ExpoDriveCurve throttleCurve(
    3,    // joystick deadband out of 127
    20,   // minimum output where drivetrain will move out of 127
    1.02 // expo curve gain
);

lemlib::ExpoDriveCurve steerCurve(
    5,    // joystick deadband out of 127
    10,   // minimum output where drivetrain will move out of 127
    1.02 // expo curve gain
);

lemlib::TrackingWheel verticalOdom(&vertical, lemlib::Omniwheel::NEW_325, 0);
lemlib::TrackingWheel horizontalOdom(&horizontal, lemlib::Omniwheel::NEW_325, 0);

lemlib::OdomSensors sensors(
	&verticalOdom,
	nullptr,
	&horizontalOdom,
	nullptr,
	&imu
);

lemlib::Chassis chassis(
	drivetrain,
	lateralPID,
	angularPID,
	sensors,
	&throttleCurve,
	&steerCurve
);

void initialize() {
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
}

void opcontrol() {
	while (true) {
		int leftY = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(leftY, rightX);
	}
}

void autonomous() {
	chassis.moveToPoint(0, 10, 800);
	chassis.turnToHeading(90, 800);
	chassis.setPose(30, 27, 75);
	
	chassis.turnToPoint(30, 25, 800);

	chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 800);
	chassis.swingToPoint(30, 25, lemlib::DriveSide::RIGHT, 800);
}