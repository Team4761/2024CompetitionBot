package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystemMock extends SubsystemBase implements VisionSubsystemInterface {
    @Override
    public Pose3d getTagPose(int id) {
        return new Pose3d();
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        return new PhotonPipelineResult();
    }

    @Override
    public int getBestTagID() {
        return 0;
    }

    @Override
    public boolean hasTargets() {
        return false;
    }

    @Override
    public void dance() {
    }
}
