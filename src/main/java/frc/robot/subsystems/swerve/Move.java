package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Move extends SwerveGoCartesianF {
    
    /**
     * <p> Makes the robot move a distance x and y
     * @param x Meters. + is forwards, - is backwards.
     * @param y Meters. + is left, - is backwards.
     */
    public Move(double x, double y) {
        super(
            Robot.getMap().swerve, new Translation2d(
                Robot.getMap().swerve.getPose().getX() + x,
                Robot.getMap().swerve.getPose().getY() + y
            )
        );
    }
}
