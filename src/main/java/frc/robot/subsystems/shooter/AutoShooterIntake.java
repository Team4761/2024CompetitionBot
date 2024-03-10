package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This is a command that runs the shooter at a speed of {speed} rotations per second for 2 seconds.
 */
public class AutoShooterIntake extends Command {
    private double speed;   // Rotations per second.
    private long timeOutTime;   // Stores the time when the command should finish executing (time out)
    private ShooterSubsystem shooter;

    /**
     * <p> This initializes the speed which the robot should shoot at in rotations per second and the time which the command should finish executing.
     * @param speed A value between -1 and 1 which defines how fast the shooter should spin in rotations per second.
     */
    public AutoShooterIntake () {
        this.speed = Robot.getShuffleboard().getSettingNum("Shooter Intake Speed");
        this.timeOutTime = System.currentTimeMillis()+8000;    // 10 second time out
        this.shooter = Robot.getMap().shooter;
    }

    /**
     * <p> Constantly sets the shooter speed to {speed} rotations per second
     */
    @Override
    public void execute() {
        shooter.setIntakeSpeed(speed);
    }

    /**
     * <p> This will end the command when the current time given by System.currentTimeMillis() equals the {endTime}
     * @return True if the 2 seconds has passed and the command is over. False if the command is still going.
     */
    @Override
    public boolean isFinished() {
        if (shooter.isPieceInUpperIntake() || timeOutTime <= System.currentTimeMillis())
            return true;
        return false;
    }

    /**
     * <p> Once the command is over, the shooter should stop spinning, so we set its speed to 0.
     */
    @Override
    public void end(boolean interrupted) {
        Robot.getMap().shooter.setIntakeSpeed(0);
    }
}
