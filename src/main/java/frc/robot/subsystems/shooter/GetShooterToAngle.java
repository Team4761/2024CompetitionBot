package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * <p> This attempts to get the shooter to a specific angle and will end once it gets within 0.01 radians of the angle.
 */
public class GetShooterToAngle extends Command {
    private double angle;
    private ShooterSubsystem shooter;

    /**
     * <p> Initializes a copy of the shooter subsystem for better performance as well as the angle it should be getting to in radians.
     * @param angleRadians The angle to get to in radians where 0 radians is fully outstretched and positive values move the shooter upwards.
     */
    public GetShooterToAngle(double angleRadians) {
        this.angle = angleRadians;
        this.shooter = Robot.getMap().shooter;
    }

    /**
     * <p> When the command is initialized, all we do is tell the ShooterSubsystem that the desired targetAngle is {angle} radians.
     * <p> The actual execution of getting the shooter to that angle is in the ShooterSubsystem's periodic method.
     */
    @Override
    public void initialize() {
        shooter.setShooterAngle(angle);
    }

    /**
     * <p> This finishes the command if the measured angle of the shooter is within 0.01 radians of the desired angle.
     * @return True if the measured angle is within 0.01 radians. False if the measured angle is too far off.
     */
    @Override
    public boolean isFinished() {
        if (Math.abs(shooter.getShooterAngle().getRadians() - angle) < 1) // If the shooter is within 0.01 radians of the target, stop.
            return true;
        return false;
    }
}
