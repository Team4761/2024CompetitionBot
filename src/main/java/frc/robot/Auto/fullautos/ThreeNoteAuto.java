package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.MoveBackCommand;
import frc.robot.Auto.ShootAuto;
import frc.robot.subsystems.intake.FullIntake;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;
import frc.robot.subsystems.swerve.SwerveTurnTo;

/**
 * <p> The robot must be placed so that when the intake drops, it falls directly on top of a note.
 * <p>
 * <p> This does 8 things:
 * <p> 1) It primes the intake by getting it down to the ground.
 * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
 * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
 * <p> 4) It runs the bottom and top intake to get the note to the actual shooter, changing the shooter's angle in the process.
 * <p> 5) It gets the shooter back to the correct angle, and then shoots.
 * <p> 6) It uses a series of move commands to get to the top note in the alliance side of notes.
 * <p> 7) It full intakes the note while turning 33 degrees to face the speaker to shoot.
 * <p> 8) It revs up the shooter and then shoots.
 * <p> 9) It backs up 1 meter.
 */
public class ThreeNoteAuto extends SequentialCommandGroup {

    /**
     * <p> The robot must be placed so that when the intake drops, it falls directly on top of a note.
     * <p>
     * <p> This does 8 things:
     * <p> 1) It primes the intake by getting it down to the ground.
     * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
     * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
     * <p> 4) It runs the bottom and top intake to get the note to the actual shooter, changing the shooter's angle in the process.
     * <p> 5) It gets the shooter back to the correct angle, and then shoots.
     * <p> 6) It uses a series of move commands to get to the top note in the alliance side of notes.
     * <p> 7) It full intakes the note while turning 33 degrees to face the speaker to shoot.
     * <p> 8) It revs up the shooter and then shoots.
     * <p> 9) It backs up 1 meter.
     */
    public ThreeNoteAuto() {
        super(
            new ShootAuto(),
            new FullIntake(Robot.getShuffleboard().getSettingNum("Intake Speed")),
            new GetShooterToAngle(Constants.SHOOTER_TWO_NOTE_SHOOT_ANGLE),
            new IntakeAndShoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed")),
            new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(0.0, -1.4)),  // Translation was found using path planner and eyeballing it
            new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(Units.degreesToRadians(35))),
            new GetShooterToAngle(Constants.SHOOTER_THREE_NOTE_SHOOT_ANGLE),
            new IntakeAndShoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed")),
            new MoveBackCommand(1.0)
        );
    }
}
