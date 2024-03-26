package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.ZeroGyro;

public class DiagonalTwoNoteAuto extends SequentialCommandGroup {

    /**
     * <p> This starts by zeroing the robot such that the backwards direction is perpendicular to the alliance wall INSTEAD of being based on the robot's current rotation.
     * <p> It must be setup so that the robot is perfectly against/facing the speaker.
     * @param isRobotOnLeftSide When looking at the robot from the alliance wall, is the robot on the left of the speaker.
     */
    public DiagonalTwoNoteAuto(boolean isRobotOnLeftSide) {
        super(
            new ZeroGyro(isRobotOnLeftSide ? -Constants.STARTING_ANGLE_DIAGONAL : Constants.STARTING_ANGLE_DIAGONAL),
            new TwoNoteAuto() // will not work
        );
    }
}
