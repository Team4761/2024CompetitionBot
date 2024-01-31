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
  private Rotation3d camerRotation = new Rotation3d(0.0, Math.PI / 12, 0.0); // Radians
  private Transform3d robotToCameraTransform = new Transform3d(cameraTranslate, camerRotation);
  Pose3d cameraPose3d = new Pose3d(cameraTranslate, camerRotation);
  private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(TagPositions.getAprilTagFieldLayout(), PoseStrategy.AVERAGE_BEST_TARGETS, mCamera, robotToCameraTransform);

  private boolean mDriverMode = false;
  public VisionSubsystem(){
    //Replace with name of cam
    mCamera = new PhotonCamera("Camera"); 

  }

  @Override
  public void periodic() {
    // var result = mCamera.getLatestResult();
    // if(result.hasTargets()){
    //   Optional<EstimatedRobotPose> estimatedRobotPose = photonEstimator.update(result);
    //   if(estimatedRobotPose.isPresent()){
    //     EstimatedRobotPose e = estimatedRobotPose.get();
    //     // RobocketsShuffleboard.addNumber("Pose X", e.estimatedPose.getX());
    //     // RobocketsShuffleboard.addNumber("Pose Y", e.estimatedPose.getY());
    //   }
    // }
    Pose3d robotPose = getPoseFromTag();
    // SmartDashboard.putNumber("X", robotPose.getX());
    // SmartDashboard.putNumber("Y", robotPose.getY());
    // SmartDashboard.putNumber("Z", robotPose.getZ());
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

  public Pose3d getPoseFromTag(){
    var result = getLatestResult();
    System.out.println("Got here");
    if(result.hasTargets()){
      //Gets ID and ambiguity (if tag is trustworthy)
      int tag = result.getBestTarget().getFiducialId();
      // SmartDashboard.putNumber("April Tag ID", tag);
      double ambiguity = result.getBestTarget().getPoseAmbiguity();
      //Get tag pose
      Optional<Pose3d> tagPoseopt = TagPositions.getTagPose3d(tag);
      if(ambiguity < .2){
        //Gets tag pose and transfroms over camera
        Pose3d tagPose = tagPoseopt.get();
        Transform3d cam2tag = result.getBestTarget().getBestCameraToTarget();
        //Cam pose on field
        Pose3d camPose = tagPose.transformBy(cam2tag.inverse());
        //Robot pose on field
        Pose3d robotPose = camPose.transformBy(robotToCameraTransform.inverse());
        return robotPose;
      } 
      else {
        // SmartDashboard.putNumber("April Tag ID", 5309);
      } 
    } 
    else {
      // SmartDashboard.putNumber("April Tag ID", 5309);
    } 
    return new Pose3d(50, 10, 10000, new Rotation3d());
  }
  private static long l = 0L;

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