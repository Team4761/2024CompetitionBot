package frc.robot.subsystems.intake;

import frc.robot.Robot;

/**
 * <p> This is the holy grail of intake commands (if it works).
 * <p> It runs both the actual intake AND the shooter intake to get a piece right up to the shooter.
 * <p> Once the piece reaches the break-beam sensor on the top of the shooter, the command ends.
 * <p> The command will also end after a 5 second time out period.
 */
public class IntakeUntilBreakbeam extends RunIntake {

    /**
     * <p> This initializes the speed and RobotMap.
     * @param speed The speed to run the intake at as a number between 0.0 to 1.0
     * @param map The RobotMap of the robot to improve performance.
     */
    
    public IntakeUntilBreakbeam(double speed) {
        super(speed);
    }

    public IntakeUntilBreakbeam(double speed, long timeout) {
        super(speed, timeout);
    }

    /**
     * <p> This checks whether or not there is a game piece at the top of the shooter and ends when the game piece gets there.
     * <p> This will also end the command after a time out of 5 seconds.
     * @return True if there is a piece in the upper shooter or if the command has been running for 5 seconds. False otherwise.
     */
    @Override
    public boolean isFinished() {
        if (Robot.getMap().intake.isPieceInIntake())
            return true;
        return super.isFinished();
    }

    /**
     * <p> Once the command ends, the shooter's and intake's speeds should be set to 0.
     * @param isInterrupted Whether or not the command was forcefully interrupted instead of reaching its isFinished ending.
     */
    @Override
    public void end(boolean isInterrupted) {
        Robot.getMap().intake.intake(0.0);
        Robot.getMap().shooter.setIntakeSpeed(0.0);
    }
}
