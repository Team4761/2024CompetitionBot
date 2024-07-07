package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class RevShooter extends Command{
    private double speed;   // Value between -1 and 1 in rotations per second
    private long length;
    private long endTime;

    /**
     * <p> This initializes the speed which the robot should shoot at in rotations per second and the time which the command should finish executing.
     * @param speed A value between -1 and 1 which defines how fast the shooter should spin in rotations per second.
     */
    public RevShooter (double speed) {
        this.speed = speed;
        
        length = 3000;
    }
    public RevShooter (double speed, double seconds) {
        this.speed = speed;
        
        length = (long)(seconds*1000);
    }
    
    @Override
    public void initialize() {
        this.endTime = System.currentTimeMillis()+length;
    }

    /**
     * <p> Constantly sets the RevShooterer speed to {speed} rotations per second
     */
    @Override
    public void execute() {
        Robot.getMap().shooter.setShooterSpeed(speed);  // Always be revving the motors
    }

    /**
     * <p> This will end the command when the current time given by System.currentTimeMillis() equals the {endTime}
     * @return True if the 2 seconds has passed and the command is over. False if the command is still going.
     */
    @Override
    public boolean isFinished() {
        if (endTime <= System.currentTimeMillis())
            return true;
        return false;
    }

    /**
     * <p> Dont stop the shooter
     */
    @Override
    public void end(boolean interrupted) {

    }
}