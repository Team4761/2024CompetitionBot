package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.ShootAuto;
import frc.robot.subsystems.intake.FullIntake;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.shooter.RevShooter;
import frc.robot.subsystems.swerve.SwerveGoThrough;
import frc.robot.subsystems.swerve.SwerveGoTo;
import frc.robot.subsystems.swerve.SwerveTurnTo;
import frc.robot.subsystems.swerve.ZeroGyro;


// numbers are still from close middle auto 
public class FarMiddleTwoNoteAuto extends SequentialCommandGroup {

    /**
     * <p> Starts against diagonal side of subwoofer further to amp and goes for the far notes in the middle
     * <p> For when a teamate has a reliable 4 note auto but dont want to risk a collision, or when you're faster then an opposing middle auto
     * @param isRobotOnLeftSide When looking at the robot from the alliance wall, is the robot on the left of the speaker.
     */
    public FarMiddleTwoNoteAuto(boolean blueAlliance) { // false when far blue, true when far red
        super(
            new ZeroGyro(blueAlliance ? Constants.STARTING_ANGLE_DIAGONAL : -Constants.STARTING_ANGLE_DIAGONAL), // make 0 degrees away from alliance wall
            new ShootAuto(),

            // note is 75 inches from center of speaker, robot is probably 40ish inches off that 
            // around 300 inches to middle of field
            new ParallelDeadlineGroup(
                new SwerveGoThrough(Robot.getMap().swerve, new Translation2d(2.3, blueAlliance ? -3 : 3), 4, 0.2), // avoid 4 note auto area
                new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(Math.PI)), // get the robot turning
                new GetShooterToAngle(Constants.SHOOTER_INTAKE_ANGLE) // prep shooter for intaking
            ),
            new ParallelDeadlineGroup(
                new SwerveGoTo(Robot.getMap().swerve, new Translation2d(7.5, blueAlliance ? -3.8 : 3.8)), // go for close middle note
                new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(Math.PI)), // get the robot so the intake faces the note
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new RunIntake(0.7, 10000) // intake the note
                )
            ),
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new SwerveGoThrough(Robot.getMap().swerve, new Translation2d(2.3, blueAlliance ? -3 : 3), 4, 0.2), // avoid far note again
                    new FullIntake(0.7, Constants.AUTO_UPTAKE_SPEED, new Rotation2d(Constants.SHOOTER_SHOOT_ANGLE))
                ),
                new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(blueAlliance ? -Constants.STARTING_ANGLE_DIAGONAL : Constants.STARTING_ANGLE_DIAGONAL)) // go back to shooting angle, 120 degrees
            ),
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new SwerveGoTo(Robot.getMap().swerve, new Translation2d(0.1, 0)), // go back to starting position
                    new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(blueAlliance ? -Constants.STARTING_ANGLE_DIAGONAL : Constants.STARTING_ANGLE_DIAGONAL)), // go back to shooting angle, 120 degrees
                    new GetShooterToAngle(Constants.SHOOTER_SHOOT_ANGLE)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(0.8),
                    new RevShooter(40, 10)
                )
            ),
            new IntakeAndShoot(40, 0.0) // shoot
            // ideally find a further point to take 2nd and 3rd shot from
        );
    }
}
