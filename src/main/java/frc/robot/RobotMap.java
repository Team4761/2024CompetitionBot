package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * <p> Contains all the subsystems for the robot to avoid static initialization order problems.
 * <p> Various subsystems will be constantly commented out for testing as not all subsystems exist on the robot at one time.
 * <p> There are some subsystems which are ONLY for testing and will not exist in the final robot.
 * <p> Throughout the code, each subsystem must be handled if it is null.
 */
public class RobotMap
{

    // Swerve

    public SwerveDriveSubsystem swerve = null;
    public VisionSubsystem vision = null;
    public IntakeSubsystem intake = null;
    public ShooterSubsystem shooter = null;
    public LedSubsystem leds = null;
    public ClimberSubsystem climber = null;

    public Rotation2d autoEndingAngle = null;   // The expected angle of the robot when auto ends

    // Gian: Ok neat system, this is not something I did on the team
    // But why is the swerve drive commented out?
    /**
     * <p> Contains all the subsystems for the robot to avoid static initialization order problems.
     * <p> Various subsystems will be constantly commented out for testing as not all subsystems exist on the robot at one time.
     * <p> There are some subsystems which are ONLY for testing and will not exist in the final robot.
     * <p> Throughout the code, each subsystem must be handled if it is null.
     */
    public RobotMap() 
    {
        intake = new IntakeSubsystem();
        swerve = new SwerveDriveSubsystem(new Translation2d(0.31115, 0.31115), new Translation2d(0.31115, -0.31115), new Translation2d(-0.31115, 0.31115), new Translation2d(-0.31115, -0.31115));    // All translations are the swerve module positions relative to the center of the bot
        //vision = new VisionSubsystem();
        shooter = new ShooterSubsystem();
        leds = new LedSubsystem();
        // climber = new ClimberSubsystem();

        // ONLY FOR TESTING
        // westcoast = new WestCoastSubsystem();
    }


    public void setAutoEndingAngle(Rotation2d endingAngle) {
        this.autoEndingAngle = endingAngle;
    }
}