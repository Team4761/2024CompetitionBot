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
import frc.robot.subsystems.swerve.SwerveGoTo;
import frc.robot.subsystems.swerve.SwerveTurnTo;
import frc.robot.subsystems.swerve.ZeroGyro;

public class CloseMiddleTwoNoteAuto extends SequentialCommandGroup {

    /**
     * <p> Starts against diagonal side of subwoofer nearest to amp and goes for the closest note in the middle
     * <p> For when a teamate has a reliable 4 note auto
     * @param isRobotOnLeftSide When looking at the robot from the alliance wall, is the robot on the left of the speaker.
     */
    public CloseMiddleTwoNoteAuto(boolean isRobotOnLeftSide) { // false when close red, true when close blue
        super(
            new ZeroGyro(isRobotOnLeftSide ? -Constants.STARTING_ANGLE_DIAGONAL : Constants.STARTING_ANGLE_DIAGONAL), // make 0 degrees away from alliance wall
            new ShootAuto(),

            // note is 75 inches from center of speaker, robot is probably 40ish inches off that 
            // around 300 inches to middle of field
            new ParallelDeadlineGroup(
                new SwerveGoTo(Robot.getMap().swerve, new Translation2d(1.3, isRobotOnLeftSide ? 1.3 : -1.3)), // avoid 4 note auto area
                new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(Math.PI)), // get the robot turning
                new GetShooterToAngle(Constants.SHOOTER_INTAKE_ANGLE) // prep shooter for intaking
            ),
            new ParallelDeadlineGroup(
                new SwerveGoTo(Robot.getMap().swerve, new Translation2d(7.5, isRobotOnLeftSide ? 1 : -1)), // go for close middle note
                new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(Math.PI)), // get the robot so the intake faces the note
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new RunIntake(0.5, 10000) // intake the note
                )
            ),
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new SwerveGoTo(Robot.getMap().swerve, new Translation2d(1.3, isRobotOnLeftSide ? 1.3 : -1.3)), // avoid far note again
                    new FullIntake(0.5, Constants.AUTO_UPTAKE_SPEED) // can take its time because long 
                ),
                new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(isRobotOnLeftSide ? -Constants.STARTING_ANGLE_DIAGONAL : Constants.STARTING_ANGLE_DIAGONAL)) // go back to shooting angle, 120 degrees
            ),
            new ParallelCommandGroup(
                new SwerveGoTo(Robot.getMap().swerve, new Translation2d()), // go back to starting position
                new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(isRobotOnLeftSide ? -Constants.STARTING_ANGLE_DIAGONAL : Constants.STARTING_ANGLE_DIAGONAL)), // go back to shooting angle, 120 degrees
                new GetShooterToAngle(Constants.SHOOTER_SHOOT_ANGLE),
                new RevShooter(40, 10)
            ),
            new IntakeAndShoot(40, 0.05) // shoot


            // go wide to avoid closest note
            // go to far note and intake
            // full intake while going back to shooting position
            // shoot
            // ideally find a further point to take 2nd and 3rd shot from
        );
    }
}
