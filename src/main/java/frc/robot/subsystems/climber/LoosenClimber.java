package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * <p> A command to let the climber extend due to the springs inside it.
 * <p> This will be used to hook the climber onto the chain as it needs to be fully extended in order to go over the chain.
 */
public class LoosenClimber extends Command {

    public LoosenClimber() {
        addRequirements(Robot.getMap().climber);
    }

    /**
     * <p> Literally all this does is make sure the climber motor is slack and the climber can extend up.
     */
    @Override
    public void initialize() {
        Robot.getMap().climber.setSpeed(0.0);
    }

    /**
     * <p> Since the command only needs to run once, isFinished will always return true
     * @return True always since the command's only purpose is to run the initialize function.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
