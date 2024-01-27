package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ResetPose extends Command {
    
    @Override
    public void initialize() {
        Robot.getMap().swerve.resetPose();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
