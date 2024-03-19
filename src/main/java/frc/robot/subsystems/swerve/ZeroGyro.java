package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This zeros the gyro to make 0 degrees/radians the direction that the physical robot is currently facing.
 */
public class ZeroGyro extends Command {

    private double offset;

    public ZeroGyro() {
        offset = 0;     // radians
    }

    public ZeroGyro(double offsetRadians) {
        this.offset = offsetRadians;
    }
    
    @Override
    public void initialize() {
        Robot.getMap().swerve.zeroGyro(new Rotation2d(offset));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
