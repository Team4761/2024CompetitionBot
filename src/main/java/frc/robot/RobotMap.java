package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystemInterface;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemInterface;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.leds.LedSubsystemInterface;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystemInterface;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystemInterface;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemInterface;
import frc.robot.subsystems.westcoast.WestCoastSubsystem;

/**
 * <p> Contains all the subsystems for the robot to avoid static initialization order problems.
 * <p> Various subsystems will be constantly commented out for testing as not all subsystems exist on the robot at one time.
 * <p> There are some subsystems which are ONLY for testing and will not exist in the final robot.
 * <p> Throughout the code, each subsystem must be handled if it is null.
 */
public class RobotMap
{
    public final SwerveDriveSubsystemInterface swerve;
    public final VisionSubsystemInterface vision;
    public final IntakeSubsystemInterface intake;
    public final ShooterSubsystemInterface shooter;
    public WestCoastSubsystem westcoast = null;
    public final LedSubsystemInterface leds;
    public final ClimberSubsystemInterface climber;

    /**
     * <p> Contains all the subsystems for the robot to avoid static initialization order problems.
     * <p> Various subsystems will be constantly commented out for testing as not all subsystems exist on the robot at one time.
     * <p> There are some subsystems which are ONLY for testing and will not exist in the final robot.
     * <p> Throughout the code, each subsystem must be handled if it is null.
     */
    public RobotMap() 
    {
        intake = IntakeSubsystem.create();
        swerve = SwerveDriveSubsystem.create(new Translation2d(0.31115, 0.31115), new Translation2d(0.31115, -0.31115), new Translation2d(-0.31115, 0.31115), new Translation2d(-0.31115, -0.31115));    // All translations are the swerve module positions relative to the center of the bot
        vision = VisionSubsystem.create();
        shooter = ShooterSubsystem.create();
        leds = LedSubsystem.create();
        climber = ClimberSubsystem.create();

        // ONLY FOR TESTING
        // westcoast = new WestCoastSubsystem();
    }
}