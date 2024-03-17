package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.MoveBackCommand;
import frc.robot.Auto.ShootAuto;
import frc.robot.subsystems.intake.FullIntake;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.swerve.Move;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;
import frc.robot.subsystems.swerve.ZeroGyro;

/**
 * <p> The robot must be placed so that when the intake drops, it falls directly on the center note on the alliance side.
 * <p>
 * <p> This does 6 things:
 * <p> 1) It primes the intake by getting it down to the ground.
 * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
 * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
 * <p> 4) It runs the bottom and top intake to get the note to the actual shooter, changing the shooter's angle in the process.
 * <p> 5) It gets the shooter back to the correct angle, and then shoots.
 * <p> 6) It moves back 1 meter.
 */
public class TwoNoteAuto extends SequentialCommandGroup {

    /**
     * <p> The robot must be placed so that when the intake drops, it falls directly on top of the center note on the alliance side.
     * <p>
     * <p> This does 6 things:
     * <p> 1) It primes the intake by getting it down to the ground.
     * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
     * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
     * <p> 4) It runs the bottom and top intake to get the note to the actual shooter, changing the shooter's angle in the process.
     * <p> 5) It gets the shooter back to the correct angle, and then shoots.
     * <p> 6) It moves back 1 meter.
     */
    public TwoNoteAuto() {
        super(
            new ZeroGyro(),
            new ShootAuto(),
            new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new GetShooterToAngle(Constants.SHOOTER_INTAKE_ANGLE),
                    new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(-2, 0))
                ),
                new RunIntake(1, 2500) //stop intaking if cartesian done and stop cartesian if 2500 timeout
            ),
            new ParallelCommandGroup(
                new FullIntake(Robot.getShuffleboard().getSettingNum("Intake Speed"), Robot.getShuffleboard().getSettingNum("Shooter Intake Speed")),
                new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(2, 0))
            ),
            new GetShooterToAngle(Constants.SHOOTER_SHOOT_ANGLE),      // Get the shooter to shooting position
            new IntakeAndShoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed"), 1), // Shoot with the speed on the shuffleboard
            new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(-2.5, 0))
        );
    }
}
