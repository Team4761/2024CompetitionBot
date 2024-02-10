package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

// ALL OF THIS IS ROUGH CODE. THE 2000 IS 2 seconds and is TEMPORARY!
public class GetShooterToAngle extends Command {
    private double angle;
    private ShooterSubsystem shooter;

    public GetShooterToAngle(double angleRadians) {
        this.angle = angleRadians;
        this.shooter = Robot.getMap().shooter;
    }

    @Override
    public void initialize() {
        shooter.setShooterAngle(angle);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        if (Math.abs(shooter.getShooterAngle() - angle) < 0.01) // If the shooter is within 0.01 radians of the target, stop.
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
