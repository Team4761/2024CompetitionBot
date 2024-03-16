package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeAndShoot extends ParallelCommandGroup {

    /**
     * Gets the shooter to the angle and then shoots.
     * @param angleRadians The angle the shooter should shoot at in radians.
     * @param speed The speed it should shoot at in rotations per second.
     */
    public IntakeAndShoot(double speed) {
        super(
            new Shoot(speed, 2),
            new SequentialCommandGroup(
                new WaitCommand(1.4),
                new ShooterIntake(1, 600),
                new PrintCommand("Finished IntakeAndShoot!")
            )
        );
    }
}
