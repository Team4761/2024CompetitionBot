package frc.robot.field;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * All positions are in inches and converted to meters with the toMeters function.
 */
public class TagPositions {

    public static AprilTagFieldLayout getAprilTagFieldLayout(){
        return AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    }

    public static Optional<Pose3d> getTagPose3d(int ID){
        return getAprilTagFieldLayout().getTagPose(ID);
    }
}
