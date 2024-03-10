package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * For incrementally stopping the shooter
 */
public class StopShooter extends Command {
    private double deccelRate;   // rotations/s^2 down until it gets to 0
    
    private long endTime;

    public StopShooter () {
        //timeout
        this.deccelRate = 2;
        endTime = System.currentTimeMillis()+1000;
    }
    /**
     * <p> This initializes the decceleration rate of the shooter | rotations/s^2.
     * @param deccelRate rotations/s^2 down until it gets to 0
     */
    public StopShooter (double deccelRate) {
        //timeout
        this.deccelRate = deccelRate;
        endTime = System.currentTimeMillis()+1000;
    }

    /**
     * <p> Constantly sets the shooter speed to {speed} rotations per second
     */
    @Override
    public void execute() {
        // if (revTime <= System.currentTimeMillis())  // If finished reving, shoot
        //     Robot.getMap().shooter.setIntakeSpeed(0.3); // Intake the piece into the shooter
        double curTarget = Robot.getMap().shooter.getTargetSpeed();
        Robot.getMap().shooter.setShooterSpeed(curTarget-Math.min(deccelRate, Math.abs(curTarget))*Math.signum(curTarget));  // Always be revving the motors
    }

    /**
     * <p> This will end the command when the current time given by System.currentTimeMillis() equals the {endTime}
     * @return True if the 2 seconds has passed and the command is over. False if the command is still going.
     */
    @Override
    public boolean isFinished() {
        if (endTime <= System.currentTimeMillis() || Robot.getMap().shooter.getTargetSpeed()==0)
            return true;
        return false;
    }

    /**
     * <p> Once the command is over, the shooter should stop spinning, so we set its speed to 0.
     */
    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            Robot.getMap().shooter.setShooterSpeed(0);
    }
}
