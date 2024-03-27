package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.ShootAuto;
import frc.robot.subsystems.intake.FullIntake;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.shooter.RevShooter;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;
import frc.robot.subsystems.swerve.SwerveTurnTo;
import frc.robot.subsystems.swerve.ZeroGyro;

public class DiagonalTwoNoteAuto extends SequentialCommandGroup {

    /**
     * <p> This starts by zeroing the robot such that the backwards direction is perpendicular to the alliance wall INSTEAD of being based on the robot's current rotation.
     * <p> It must be setup so that the robot is perfectly against/facing the speaker.
     * @param isRobotOnLeftSide When looking at the robot from the alliance wall, is the robot on the left of the speaker.
     */
    public DiagonalTwoNoteAuto(boolean isRobotOnLeftSide) {
        super(
            new ZeroGyro(isRobotOnLeftSide ? -Constants.STARTING_ANGLE_DIAGONAL : Constants.STARTING_ANGLE_DIAGONAL),
            new ShootAuto(), // shoot first note

            // move back and intake 2nd note
            
            new ParallelCommandGroup(
                new GetShooterToAngle(Constants.SHOOTER_INTAKE_ANGLE),
                new ParallelDeadlineGroup(
                    new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(2, isRobotOnLeftSide ? -0.15 : 0.15)), // forwards should be away from alliance wall with gyro offset
                    new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(Math.PI)),
                    new RunIntake(Constants.AUTO_INTAKE_SPEED, 3000)
                )
            ),

            // go back while full intaking and returning to starting angle
            new ParallelCommandGroup(
                new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(0.15, isRobotOnLeftSide ? -0.05 : 0.05)),
                new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(isRobotOnLeftSide ? Constants.STARTING_ANGLE_DIAGONAL : -Constants.STARTING_ANGLE_DIAGONAL)),
                new FullIntake(Constants.AUTO_INTAKE_SPEED, Constants.AUTO_UPTAKE_SPEED)
            ),
            
            // get shooter to angle and rev
            new ParallelDeadlineGroup(
                new GetShooterToAngle(Constants.SHOOTER_SHOOT_ANGLE),      // Get the shooter to shooting position
                new RevShooter(40, 4) // rev motor while shooter is angling
            ),

            new IntakeAndShoot(40, 0.5)
        );
    }
}
