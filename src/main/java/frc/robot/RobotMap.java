package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveModuleNeo;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.westcoast.WestCoastSubsystem;

// Contains all physical items on the robot (motors, encoders, LEDs, etc)
public class RobotMap
{

    // Swerve
    // Gian: why are these neos here if we end up just making new ones in the swervedrivesubsystem?
    public SwerveModuleNeo swerve_frontLeftModule;
    public SwerveModuleNeo swerve_frontRightModule;
    public SwerveModuleNeo swerve_backLeftModule;
    public SwerveModuleNeo swerve_backRightModule;

    public SwerveDriveSubsystem swerve = null;
    public VisionSubsystem vision = null;
    public IntakeSubsystem intake = null;
    public ShooterSubsystem shooter = null;
    public WestCoastSubsystem westcoast = null;
    public LedSubsystem leds = null;

    // Gian: Ok neat system, this is not something I did on the team
    // But why is the swerve drive commented out?
    public RobotMap() 
    {
        // intake = new IntakeSubsystem();
        swerve = new SwerveDriveSubsystem(new Translation2d(-0.31115, 0.31115), new Translation2d(0.31115, 0.31115), new Translation2d(-0.31115, -0.31115), new Translation2d(0.31115, -0.31115));    // All translations are the swerve module positions relative to the center of the bot
        // vision = new VisionSubsystem();
        //shooter = new ShooterSubsystem();
        // leds=new LedSubsystem();

        // ONLY FOR TESTING
        // westcoast = new WestCoastSubsystem();
    }
}