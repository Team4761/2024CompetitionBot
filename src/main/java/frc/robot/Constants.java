package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * <p> This contains all the physical constants and port numbers on the robot.
 * <p> However, many PID values and Feedforward values are stored within their respective subsystem to make it easier to tune and understand the values.
 */
public class Constants
{
    public static int DRIVE_CONTROLLER_PORT = 0;   // On Dashboard under controller section
    public static int SHOOTER_CONTROLLER_PORT = 1;  // On Dashboard under controller section

    // Auto Constants, because may differ from driver preferences
    public static double AUTO_UPTAKE_SPEED = 0.16;
    public static double AUTO_INTAKE_SPEED = 0.95;
    public static double AUTO_SHOOT_ANGLE = Units.degreesToRadians(53);

    // Swerve
    public static int FL_DRIVE_PORT = 20;    // CAN Port.
    public static int FR_DRIVE_PORT = 22;    // CAN Port.
    public static int BL_DRIVE_PORT = 23;    // CAN Port.
    public static int BR_DRIVE_PORT = 21;    // CAN Port.

    public static int FL_ROTATE_PORT = 5;   // CAN Port.
    public static int FR_ROTATE_PORT = 7;   // CAN Port.
    public static int BL_ROTATE_PORT = 8;   // CAN Port.
    public static int BR_ROTATE_PORT = 6;   // CAN Port.

    public static int FL_ENCODER_PORT = 27;  // DIO Port?
    public static int FR_ENCODER_PORT = 26;  // DIO Port?
    public static int BL_ENCODER_PORT = 24;  // DIO Port?
    public static int BR_ENCODER_PORT = 25;  // DIO Port?

    // Intake (currently conflicts with Swerve ports)
    public static int INTAKE_TOP_PORT = 1;        // CAN Port.
    public static int INTAKE_BOT_PORT = 30;         // CAN Port. 
    public static int INTAKE_ANGLE_LEFT_MOTOR_PORT = 4;   // CAN Port.

    // Shooter
    public static int SHOOTER_LEFT_MOTOR_PORT = 15;  // CAN Port.
    public static int SHOOTER_RIGHT_MOTOR_PORT = 16;  // CAN Port.
    public static int SHOOTER_INTAKE_MOTOR_PORT = 2;  // CAN Port. 
    public static int SHOOTER_ANGLE_RIGHT_MOTOR_PORT = 11;  // CAN Port.
    public static int SHOOTER_SENSOR_UPPER_PORT = 1;    // DIO Port. breakbeams
    public static int SHOOTER_SENSOR_LOWER_PORT = 0;    // DIO Port.
    //public static int BREAKBEAM_IS_BROKEN = 0;

    public static double ENCODER_UNITS_TO_RADIANS = Math.PI * 2;  // Based on the motor's current position, this converts the native units to the rotation in radians.
    public static double SHOOTER_RPM_TO_MPS = 0.001;        // PLACEHOLDER. This converts the speed which the shooter is angling itself from rotations per minute to meters per second

    // Climber
    public static int CLIMBER_MOTOR_PORT = -1;  // CAN Port.

    // Physical Constants
    // TODO: find these constants
    public static double DRIVETRAIN_WHEEL_DIAMETER_M = 0.0991; // wheel diameter in meters. This is 3.9 inches
    public static double DRIVETRAIN_WHEEL_CIRCUMFERENCE_M = DRIVETRAIN_WHEEL_DIAMETER_M*Math.PI; // wheel circumference in meters
    public static double DRIVETRAIN_MAX_SPEED_MPS = 4.0; // random number! :D
    public static double DRIVETRAIN_GEAR_RATIO = 6.12; //gear ratio in L3 swerve
    public static double RPM_TO_MPS_CONVERSION = DRIVETRAIN_WHEEL_CIRCUMFERENCE_M / 60.0;
    public static double SPARKMAX_UNITS_PER_ROTATION = 42;
    public static double DRIVETRAIN_ENCODER_UNITS_TO_METERS = DRIVETRAIN_WHEEL_CIRCUMFERENCE_M / SPARKMAX_UNITS_PER_ROTATION / DRIVETRAIN_GEAR_RATIO;    // The scale factor for the drivetrain wheels encoder units to meters

    // Kraken Swerve Drive
    public static double KRAKEN_RPM_TO_MPS_CONVERSION = DRIVETRAIN_WHEEL_CIRCUMFERENCE_M / 60.0;

    // Swerve driving acceleration limit (m/s^2)
    public static double SWERVE_ACCELERATION_LIMIT = 3; 

    // Swerve PID Control
    public static double SWERVE_P = 2;    // The higher this value is, the longer it takes for the robot to start slowing down
    public static double SWERVE_I = 0.0;    // The higher this value is, the more speed it gains overtime
    public static double SWERVE_D = 0.0;    // WIP, not implemented yet

    public static double SWERVE_ROTATE_P = 6;

    // Position Constants
    public static double INTAKE_START_POSITION = Units.degreesToRadians(100.0); // In radians
    public static double INTAKE_INTAKE_POSITION = Units.degreesToRadians(23); // In radians
    public static double INTAKE_ACTIVE_DRIVING_POSITION = Units.degreesToRadians(290);  // In radians

    public static double SHOOTER_START_ANGLE = Units.degreesToRadians(70);  // In radians
    public static double SHOOTER_INTAKE_ANGLE = Units.degreesToRadians(50); // In radians
    public static double SHOOTER_IDLE_ANGLE = Units.degreesToRadians(50);  // In radians
    public static double SHOOTER_SHOOT_ANGLE = Units.degreesToRadians(60);  // In radians (Is untuned WIP)
    public static double SHOOTER_TWO_NOTE_SHOOT_ANGLE = Units.degreesToRadians(50); // In radians (Is untuned WIP)
    public static double SHOOTER_THREE_NOTE_SHOOT_ANGLE = Units.degreesToRadians(47); // In radians (Is untuned WIP)

    public static double STARTING_ANGLE_DIAGONAL = Units.degreesToRadians(120); // 120 degree offset to have forwards be away from alliance wall


    // LEDSSSS
    public static int LED_PORT = 6; // DIO Port.
}