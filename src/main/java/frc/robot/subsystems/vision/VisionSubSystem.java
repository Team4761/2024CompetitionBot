// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.field.TagPositions;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubSystem extends SubsystemBase {
  PhotonCamera mCamera;
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;
  double mCameraHeight = .19;
  double mCameraPitch = Units.degreesToRadians(30);
  private final PhotonPoseEstimator poseEstimator;
  private double lastEstTimestamp = 0;
  private boolean mDriverMode = false;
  //DOCS - https://docs.photonvision.org/en/latest/docs/simulation/simulation.html


  public VisionSubSystem(){
        mCamera = new PhotonCamera("Cam");
        //Configures Pose Estimator
        poseEstimator = new PhotonPoseEstimator(TagPositions.getFieldTagLayout(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mCamera, Constants.robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if(Robot.isSimulation()){
            //Creates vision simulation with name "Vision Simulation" on Network Tables
            visionSim = new VisionSystemSim("Vision Simulation");

            //Adds april tags to sim
            visionSim.addAprilTags(TagPositions.getFieldTagLayout());

            //Adds camera properties
            var cameraProp = new SimCameraProperties();
            //TODO change to correct camera properties
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);

            cameraSim = new PhotonCameraSim(mCamera, cameraProp);
            visionSim.addCamera(cameraSim, Constants.robotToCamera);

            cameraSim.enableDrawWireframe(true); 
        }   
  }

  @Override
  public void periodic() {
    
  }

  public PhotonPipelineResult getLatestResult(){
    return mCamera.getLatestResult();
  }
     /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
    var visionEst = poseEstimator.update();
    double latestTimestamp = mCamera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                    est ->
                            getSimDebugField()
                                    .getObject("VisionEstimation")
                                    .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                    });
        }
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
  }


  public String toString(){
    var result = mCamera.getLatestResult();
    if(mDriverMode){
      return "Driver mode";
    }
    else if(result.hasTargets()){
      String ids = "";
      List<PhotonTrackedTarget> targets = result.getTargets();
      for(PhotonTrackedTarget target : targets){
        ids += target.getFiducialId();
      }
      return ids;
    }
    else{
      return "No tag present";
    }
  }

  





    //Simulation
    public void simulationPeriodic(Pose2d robotSimPose){
        visionSim.update(robotSimPose);
    }

    public void resetSimPose(Pose2d pose){
        if(Robot.isSimulation()){
          visionSim.resetRobotPose(pose);
        }
    }
     public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}