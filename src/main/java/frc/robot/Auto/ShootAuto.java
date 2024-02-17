package frc.robot.Auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.intake.GetIntakeToSetPosition;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.Shoot;

public class ShootAuto extends SequentialCommandGroup {

    public ShootAuto() {
        super(
                new GetIntakeToSetPosition(Units.degreesToRadians(0)),  // Move the intake down
                new GetShooterToAngle(Units.degreesToRadians(63)),      // Get the shooter to shooting position
                new Shoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed")), // Shoot with the speed on the shuffleboard
                new MoveBackCommand(1.2) // Move 2 meters back
        );
    }
}
