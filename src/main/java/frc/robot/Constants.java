package frc.robot;


/**
 * <p> This contains all the physical constants and port numbers on the robot.
 * <p> However, many PID values and Feedforward values are stored within their respective subsystem to make it easier to tune and understand the values.
 */
public class Constants
{
    public static int DRIVE_CONTROLLER_PORT = 0;   // On Dashboard under controller section
    public static int SHOOTER_CONTROLLER_PORT = 1;  // On Dashboard under controller section

    // Swerve
    public static int FL_DRIVE_PORT = 1;    // CAN Port.
    public static int FR_DRIVE_PORT = 8;    // CAN Port.
    public static int BL_DRIVE_PORT = 2;    // CAN Port.
    public static int BR_DRIVE_PORT = 6;    // CAN Port.

    public static int FL_ROTATE_PORT = 4;   // CAN Port.
    public static int FR_ROTATE_PORT = 5;   // CAN Port.
    public static int BL_ROTATE_PORT = 3;   // CAN Port.
    public static int BR_ROTATE_PORT = 7;   // CAN Port.

    public static int FL_ENCODER_PORT = 1;  // DIO Port?
    public static int FR_ENCODER_PORT = 4;  // DIO Port?
    public static int BL_ENCODER_PORT = 3;  // DIO Port?
    public static int BR_ENCODER_PORT = 2;  // DIO Port?

    // Intake (currently conflicts with Swerve ports)
    public static int INTAKE_RIGHT_PORT = 1;        // CAN Port.
    public static int INTAKE_LEFT_PORT = 3;         // CAN Port.
    public static int INTAKE_ANGLE_LEFT_MOTOR_PORT = -1;   // CAN Port.
    public static int INTAKE_ANGLE_RIGHT_MOTOR_PORT = -1;   // CAN Port.

    // Shooter
    public static int SHOOTER_LEFT_MOTOR_PORT = 15;  // CAN Port.
    public static int SHOOTER_RIGHT_MOTOR_PORT = 16;  // CAN Port.
    public static int SHOOTER_INTAKE_LEFT_MOTOR_PORT = 5;  // CAN Port.
    public static int SHOOTER_INTAKE_RIGHT_MOTOR_PORT = 3;  // CAN Port.
    public static int SHOOTER_ANGLE_LEFT_MOTOR_PORT = -1;   // CAN Port.
    public static int SHOOTER_ANGLE_RIGHT_MOTOR_PORT = -1;  // CAN Port.
    public static int SHOOTER_SENSOR_UPPER_PORT = 0;    // DIO Port.
    public static int SHOOTER_SENSOR_LOWER_PORT = 1;    // DIO Port.

    public static double NEO_UNITS_TO_RADIANS = 0.0174533;  // Based on the motor's current position, this converts the native units to the rotation in radians.
    public static double SHOOTER_RPM_TO_MPS = 0.001;        // PLACEHOLDER. This converts the speed which the shooter is angling itself from rotations per minute to meters per second

    // Climber
    public static int CLIMBER_MOTOR_PORT = -1;  // CAN Port.

    // Physical Constants
    // TODO: find these constants
    public static double DRIVETRAIN_WHEEL_DIAMETER_M = 0.1016; // wheel diameter in meters. This is 4 inches btw
    public static double DRIVETRAIN_WHEEL_CIRCUMFERENCE_M = DRIVETRAIN_WHEEL_DIAMETER_M*Math.PI; // wheel circumference in meters
    public static double DRIVETRAIN_MAX_SPEED_MPS = 4.0; // random number! :D
    public static double DRIVETRAIN_GEAR_RATIO = 6.75; //random number!
    public static double RPM_TO_MPS_CONVERSION = DRIVETRAIN_WHEEL_CIRCUMFERENCE_M / 60.0;
    public static double SPARKMAX_UNITS_PER_ROTATION = 42;
    public static double DRIVETRAIN_ENCODER_UNITS_TO_METERS = DRIVETRAIN_WHEEL_CIRCUMFERENCE_M / SPARKMAX_UNITS_PER_ROTATION / DRIVETRAIN_GEAR_RATIO;    // The scale factor for the drivetrain wheels encoder units to meters

    // Kraken Swerve Drive
    public static double KRAKEN_RPM_TO_MPS_CONVERSION = DRIVETRAIN_WHEEL_CIRCUMFERENCE_M / 60.0;

    // Swerve PID Control
    public static double SWERVE_P = 5.0;    // The higher this value is, the longer it takes for the robot to start slowing down
    public static double SWERVE_I = 0.5;    // The higher this value is, the more speed it gains overtime
    public static double SWERVE_D = 0.0;    // WIP, not implemented yet
}