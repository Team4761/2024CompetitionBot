package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveModuleNeo;
import frc.robot.subsystems.vision.VisionSubsystem;

// Contains all physical items on the robot (motors, encoders, LEDs, etc)
public class RobotMap
{
    // Swerve
    public SwerveModuleNeo swerve_frontLeftModule;
    public SwerveModuleNeo swerve_frontRightModule;
    public SwerveModuleNeo swerve_backLeftModule;
    public SwerveModuleNeo swerve_backRightModule;

    public SwerveDriveSubsystem swerve = null;
    public VisionSubsystem vision = null;
    public IntakeSubsystem intake = null;
    public ShooterSubsystem shooter = null;


    public RobotMap() 
    {
        try {
            swerve = new SwerveDriveSubsystem(new Translation2d(-12.25, 12.25), new Translation2d(12.25, 12.25), new Translation2d(-12.25, -12.25), new Translation2d(12.25, -12.25));
        } catch (Exception e) {
            System.out.println(e);
        }
        try {
            vision = new VisionSubsystem();
        } catch (Exception e) {
            System.out.println(e);
        }
        try {
            //intake = new IntakeSubsystem();
        } catch (Exception e) {
            System.out.println(e);
        }
        try {
            //shooter = new ShooterSubsystem();
        } catch (Exception e) {
            System.out.println(e);
        }
    }
}