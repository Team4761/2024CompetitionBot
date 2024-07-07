package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootAtAngle extends SequentialCommandGroup {

    /**
     * Gets the shooter to the angle and then shoots.
     * @param angleRadians The angle the shooter should shoot at in radians.
     * @param speed The speed it should shoot at in rotations per second.
     */
    public ShootAtAngle(double angleRadians, double speed) {
        super(
            new GetShooterToAngle(angleRadians),
            new IntakeAndShoot(speed)
        );
    }
}
