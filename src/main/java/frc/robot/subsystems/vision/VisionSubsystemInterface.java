package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionSubsystemInterface extends Subsystem {
    Pose3d getTagPose(int id);

    PhotonPipelineResult getLatestResult();

    int getBestTagID();

    boolean hasTargets();

    void dance();
}
