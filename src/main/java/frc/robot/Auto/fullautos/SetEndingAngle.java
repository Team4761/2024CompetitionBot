package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * <p> Once the auto finishes, this sets the final angle of the robot in order to rezero the gyro!
 * <p> 0 degrees is the robot looking towards the center of the field. The positive direction is rotating towards the right (top down view, rotating clockwise)
 * <p> The rezero-ing is done in teleopInit.
 */
public class SetEndingAngle extends Command {
    
    private Rotation2d endingAngle;


    /**
     * <p> Once the auto finishes, this sets the final angle of the robot in order to rezero the gyro!
     * <p> 180 degrees is the robot looking towards the center of the field. The positive direction is rotating towards the right (top down view, rotating clockwise)
     * <p> The rezero-ing is done in teleopInit.
     */
    public SetEndingAngle(Rotation2d endingAngle) {
        this.endingAngle = endingAngle;
    }

    @Override
    public void initialize() {
        Robot.getMap().setAutoEndingAngle(endingAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
