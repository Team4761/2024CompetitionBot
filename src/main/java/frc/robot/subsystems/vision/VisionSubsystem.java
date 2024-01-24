// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.proto.Transform3dProto;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobocketsShuffleboard;
import frc.robot.field.TagPositions;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  //**ALL ANGLES IN RADIANS ALL DISTANCES IN METERS**//
  PhotonCamera mCamera;

  private Translation3d cameraTranslate = new Translation3d(.3915,0.0,0.35); // Meters
  private Rotation3d camerRotation = new Rotation3d(Math.PI/12.0, 0.0, 0.0); // Radians
  private Transform3d cameraTransform = new Transform3d(cameraTranslate, camerRotation);
  Pose3d cameraPose3d = new Pose3d(cameraTranslate, camerRotation);
  private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(TagPositions.getAprilTagFieldLayout(), PoseStrategy.AVERAGE_BEST_TARGETS, mCamera, cameraTransform);
  double mCameraHeight = .19;
  double mCameraPitch = Units.degreesToRadians(30);

  private boolean mDriverMode = false;
  public VisionSubsystem(){
    //Replace with name of cam
    mCamera = new PhotonCamera("Camera"); 

  }

  @Override
  public void periodic() {
    var result = mCamera.getLatestResult();
    if(result.hasTargets()){
      Optional<EstimatedRobotPose> estimatedRobotPose = photonEstimator.update(result);
      if(estimatedRobotPose.isPresent()){
        EstimatedRobotPose e = estimatedRobotPose.get();
        RobocketsShuffleboard.addNumber("Pose X", e.estimatedPose.getX());
        RobocketsShuffleboard.addNumber("Pose Y", e.estimatedPose.getY());
      }
    }
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