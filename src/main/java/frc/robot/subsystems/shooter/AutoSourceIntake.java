package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This is a command that runs the shooter at a speed of {speed} rotations per second for 2 seconds.
 */
public class AutoSourceIntake extends Command {
    private double intakeSpeed;   // Rotations per second.
    private double shooterSpeed;
    private long timeOutTime;   // Stores the time when the command should finish executing (time out)
    private ShooterSubsystem shooter;
    private boolean trued; // needs to hit the bottom breakbeam then go past

    /**
     * <p> This initializes the speed which the robot should shoot at in rotations per second and the time which the command should finish executing.
     * @param speed A value between -1 and 1 which defines how fast the shooter should spin in rotations per second.
     */
    public AutoSourceIntake () {
        this.intakeSpeed = -Robot.getShuffleboard().getSettingNum("Shooter Outtake Speed");
        this.shooterSpeed = -Robot.getShuffleboard().getSettingNum("Shooter In Speed");
        this.timeOutTime = System.currentTimeMillis()+8000;    // 10 second time out
        this.shooter = Robot.getMap().shooter;
    }

    /**
     * <p> Constantly sets the shooter speed to {speed} rotations per second
     */
    @Override
    public void execute() {
        shooter.setIntakeSpeed(intakeSpeed);
        shooter.setShooterSpeed(shooterSpeed);
        if (shooter.isPieceInLowerIntake()) {
            trued = true;
        }
    }






    /**
     * <p> This will end the command when the current time given by System.currentTimeMillis() equals the {endTime}
     * @return True if the 2 seconds has passed and the command is over. False if the command is still going.
     */
    @Override
    public boolean isFinished() {
        if ((!shooter.isPieceInLowerIntake() && trued)||timeOutTime <= System.currentTimeMillis()) // if it went past the lower intake
            return true;
        return false;
    }

    /**
     * <p> Once the command is over, the shooter should stop spinning, so we set its speed to 0.
     */
    @Override
    public void end(boolean interrupted) {
        Robot.getMap().shooter.setIntakeSpeed(0);
        Robot.getMap().shooter.setShooterSpeed(0);
    }
}
