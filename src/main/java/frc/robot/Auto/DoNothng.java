package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DoNothng extends Command {
    
    @Override
    public void initialize() {
        Robot.getMap().swerve.swerveDriveF(0,0,0,false);
    }
}
