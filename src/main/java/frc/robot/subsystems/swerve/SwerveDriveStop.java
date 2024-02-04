package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SwerveDriveStop extends Command {

    public SwerveDriveStop () {}

    @Override
    public void initialize() {
        Robot.getMap().swerve.swerveDriveF(0,0,0,false);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
