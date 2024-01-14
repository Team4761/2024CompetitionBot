package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

// ALL OF THIS IS ROUGH CODE. THE 2000 IS 2 seconds and is TEMPORARY!
public class Shoot extends Command {
    private double speed; // Value between -1 and 1
    private long endTime;

    public Shoot (double speed) {
        this.speed = speed;
        this.endTime = System.currentTimeMillis()+2000;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Robot.getMap().shooter.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        if (endTime <= System.currentTimeMillis())
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.getMap().shooter.setSpeed(0);
    }
}
