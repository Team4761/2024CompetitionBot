package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This zeros the gyro to make 0 degrees/radians the direction that the physical robot is currently facing.
 */
public class ZeroGyro extends Command {
    
    @Override
    public void initialize() {
        Robot.getMap().swerve.zeroGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
