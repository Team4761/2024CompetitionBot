package frc.robot.Auto.fullautos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Auto.ShootAuto;
import frc.robot.subsystems.swerve.ZeroGyro;

public class FarTwoNoteAuto extends SequentialCommandGroup {

    /**
     * <p> Starts against diagonal side of subwoofer nearest to amp and goes for the closest note in the middle
     * <p> For when a teamate has a reliable 4 note auto
     * @param isRobotOnLeftSide When looking at the robot from the alliance wall, is the robot on the left of the speaker.
     */
    public FarTwoNoteAuto(boolean isRobotOnLeftSide) {
        super(
            new ZeroGyro(isRobotOnLeftSide ? Constants.STARTING_ANGLE_DIAGONAL : -Constants.STARTING_ANGLE_DIAGONAL),
            new ShootAuto()
            // go wide to avoid closest note
            // go to far note and intake
            // full intake while going back to shooting position
            // shoot
            // 
        );
    }
}
