package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.MoveBackCommand;
import frc.robot.Auto.ShootAuto;
import frc.robot.subsystems.swerve.ZeroGyro;

/**
 * <p> This does 4 things:
 * <p> 1) It primes the intake by getting it down to the ground.
 * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
 * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
 * <p> 4) It then moves back 1 meter.
 */
public class OneNoteAuto extends SequentialCommandGroup {

    /**
     * <p> This does 4 things:
     * <p> 1) It primes the intake by getting it down to the ground.
     * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
     * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
     * <p> 4) It then moves back 1 meter.
     */
    public OneNoteAuto() {
        super(
            new ZeroGyro(),
            new ShootAuto(),
            new MoveBackCommand(3.0),
            new SetEndingAngle(new Rotation2d(Units.degreesToRadians(180)))
        );
    }
}
