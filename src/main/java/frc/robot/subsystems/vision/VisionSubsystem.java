// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  //**ALL ANGLES IN RADIANS ALL DISTANCES IN METERS**//
  PhotonCamera mCamera;
  // Initialization has been moved to RobotMap
  //private static final VisionSubsystem INSTANCE = new VisionSubsystem();
  //public static VisionSubsystem getInstance(){
  //  return INSTANCE;
  //}
  double mCameraHeight = .19;
  double mCameraPitch = Units.degreesToRadians(30);

  private boolean mDriverMode = false;
  private AprilTagFieldLayout tagFieldLayout;

  public VisionSubsystem(){
    //Replace with name of cam
    mCamera = new PhotonCamera("Camera");
    tagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  @Override
  public void periodic() {
    
  }

  public Pose3d getTagPose(int id) {
    return tagFieldLayout.getTagPose(id).orElse(null);
  }

  public PhotonPipelineResult getLatestResult(){
    return mCamera.getLatestResult();
  }

  public int getBestTagID(){
     return mCamera.getLatestResult().getBestTarget().getFiducialId();
   }

  public boolean hasTargets(){
    return mCamera.getLatestResult().hasTargets();
  }
  public Translation2d getTransDiff(double targetHeight){
    if(hasTargets()){
      double pitch = Units.degreesToRadians(mCamera.getLatestResult().getBestTarget().getPitch());
      double distance = PhotonUtils.calculateDistanceToTargetMeters(
        mCameraHeight, 
        targetHeight, 
        mCameraPitch,
        pitch);
      double yaw = Units.degreesToRadians(mCamera.getLatestResult().getBestTarget().getYaw());
      double yDiff = distance * Math.cos(pitch);
      double xDiff = yDiff / Math.cos(yaw) * Math.sin(yaw);
      return new Translation2d(xDiff, yDiff - 1);
    }
    else{
      return new Translation2d();
    }
  }

  

  public void toggleDriverMode(){
    mDriverMode = !mDriverMode;
    mCamera.setDriverMode(mDriverMode);
  }
  //Returns false if not in driver mode
  public boolean getDriverMode(){
    return mCamera.getDriverMode();
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
} 