package frc.robot;

import frc.robot.subsystems.swerve.SwerveModuleNeo;

// Contains all physical items on the robot (motors, encoders, LEDs, etc)
public class RobotMap
{
    // Swerve
    public SwerveModuleNeo swerve_frontLeftModule  = new SwerveModuleNeo(1 , 4 , 1 , -98.5, -1.0,  1.0);
    public SwerveModuleNeo swerve_frontRightModule = new SwerveModuleNeo(8 , 5 , 4 , -6, -1.0, -1.0);
    public SwerveModuleNeo swerve_backLeftModule   = new SwerveModuleNeo(2 , 3 ,3 , 112, -1.0, -1.0);
    public SwerveModuleNeo swerve_backRightModule  = new SwerveModuleNeo(6 , 7 , 2 , -53, -1.0,  1.0);
}