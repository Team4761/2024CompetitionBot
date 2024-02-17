package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * <p> A command to tighten the climber (bring it closer together)
 * <p> This will also be used to actually climb. Once the climber is hooked up to the chain, it should be tightened to lift the robot up.
 */
public class TightenClimber extends Command {
    public TightenClimber() {
        addRequirements(Robot.getMap().climber);
    }

    /**
     * <p> Literally all this does is make sure the climber motor speed is constantly running to pull it inside.
     */
    @Override
    public void initialize() {
        Robot.getMap().climber.setSpeed(1.0);
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
