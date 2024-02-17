package frc.robot.Auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.GetIntakeToSetPosition;
import frc.robot.subsystems.shooter.GetShooterToAngle;

public class PrepShooterAuto extends SequentialCommandGroup {
    public PrepShooterAuto() {
        super(
            new GetIntakeToSetPosition(Units.degreesToRadians(0)),  // Move the intake down
            new GetShooterToAngle(Units.degreesToRadians(63)),      // Get the shooter to shooting position
            new MoveBackCommand(1.2) // Move 2 meters back
        );
    }
}
