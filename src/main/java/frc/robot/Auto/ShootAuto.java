package frc.robot.Auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.GetIntakeToSetPosition;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.shooter.Shoot;

/**
 * <p> This does 3 things:
 * <p> 1) It primes the intake by getting it down to the ground.
 * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
 * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
 */
public class ShootAuto extends SequentialCommandGroup {

    /**
     * <p> This does 3 things:
     * <p> 1) It primes the intake by getting it down to the ground.
     * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
     * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
    */
    public ShootAuto() {
        super(
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new GetIntakeToSetPosition(Units.degreesToRadians(Constants.INTAKE_INTAKE_POSITION)),  // Move the intake down
                    new GetShooterToAngle(Constants.SHOOTER_SHOOT_ANGLE)      // Get the shooter to shooting position
                ),
                new Shoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed")) // rev until intake and shooter position are done
            ),
            new IntakeAndShoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed"), 0.5) // Shoot with the speed on the shuffleboard
        );
    }
}
