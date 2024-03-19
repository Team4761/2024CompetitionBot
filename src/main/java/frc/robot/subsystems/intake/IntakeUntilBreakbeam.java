package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * <p> This is the holy grail of intake commands (if it works).
 * <p> It runs both the actual intake AND the shooter intake to get a piece right up to the shooter.
 * <p> Once the piece reaches the break-beam sensor on the top of the shooter, the command ends.
 * <p> The command will also end after a 5 second time out period.
 */
public class IntakeUntilBreakbeam extends Command {
    private RobotMap map;   // The robot's RobotMap to access the intake and shooter subsystems.
    private long timeOut;   // The time at which the command should end.
    private long length = 5000;

    /**
     * <p> This initializes the speed and RobotMap.
     * @param speed The speed to run the intake at as a number between 0.0 to 1.0
     * @param map The RobotMap of the robot to improve performance.
     */
    
    public IntakeUntilBreakbeam() {
    }

    public IntakeUntilBreakbeam(long timeout) {
        length = System.currentTimeMillis()+timeout;
    }

    @Override
    public void initialize() {
        this.timeOut = System.currentTimeMillis() + length;
    }

    /**
     * <p> This runs both the intake's intake and the shooter's intake at a speed of {speed}
     */
    @Override
    public void execute() {
        map.intake.intake(Robot.getShuffleboard().getSettingNum("Intake Speed"));
    }

    /**
     * <p> This checks whether or not there is a game piece at the top of the shooter and ends when the game piece gets there.
     * <p> This will also end the command after a time out of 5 seconds.
     * @return True if there is a piece in the upper shooter or if the command has been running for 5 seconds. False otherwise.
     */
    @Override
    public boolean isFinished() {
        if (map.intake.isPieceInIntake())
            return true;
        if (timeOut <= System.currentTimeMillis())
            return true;
        return false;
    }

    /**
     * <p> Once the command ends, the shooter's and intake's speeds should be set to 0.
     * @param isInterrupted Whether or not the command was forcefully interrupted instead of reaching its isFinished ending.
     */
    @Override
    public void end(boolean isInterrupted) {
        map.intake.intake(0.0);
        map.shooter.setIntakeSpeed(0.0);
    }
}
