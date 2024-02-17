// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase implements VisionSubsystemInterface {

  //**ALL ANGLES IN RADIANS ALL DISTANCES IN METERS**//
  PhotonCamera mCamera;
  private AprilTagFieldLayout tagFieldLayout;
  private PhotonPoseEstimator estimator;
  private Transform3d robotToCamera;

  private double poseX = 0.0;
  private double poseY = 0.0;

  private RobotMap map = Robot.getMap();

  public static VisionSubsystemInterface create() {
    try {
      return new VisionSubsystem();
    } catch (Throwable t) {
      t.printStackTrace();
      return new VisionSubsystemMock();
    }
  }

  public VisionSubsystem(){
    //Replace with name of cam
    mCamera = new PhotonCamera("Camera");
    tagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    robotToCamera = new Transform3d(.38, -.12, .125, new Rotation3d(0, Units.degreesToRadians(15), 0));
    estimator = new PhotonPoseEstimator(tagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mCamera, robotToCamera);
  }

  @Override
  public void periodic() {
    var result = mCamera.getLatestResult();
    if(result.hasTargets()){
      Optional<EstimatedRobotPose> estimatedRobotPose = estimator.update(result);
      if(estimatedRobotPose.isPresent()){
        EstimatedRobotPose e = estimatedRobotPose.get();
        SmartDashboard.putNumber("Pose X", e.estimatedPose.getX());
        SmartDashboard.putNumber("Pose Y", e.estimatedPose.getY());
        poseX = e.estimatedPose.getX();
        poseY = e.estimatedPose.getY();
      }
    }
  }

  @Override
  public Pose3d getTagPose(int id) {
    return tagFieldLayout.getTagPose(id).orElse(null);
  }

  @Override
  public PhotonPipelineResult getLatestResult(){
    return mCamera.getLatestResult();
  }

  @Override
  public int getBestTagID(){
     return mCamera.getLatestResult().getBestTarget().getFiducialId();
   }

  @Override
  public boolean hasTargets(){
    return mCamera.getLatestResult().hasTargets();
  }

  @Override
  public void dance(){
    double x = 0.0;
    double y = 0.0;
    double[] smoothX = new double[]{0,0,0,0,0};
    double[] smoothY = new double[]{0,0,0,0,0};
    int currentSmooth = 0;

      smoothX[currentSmooth] = poseX;
      smoothY[currentSmooth] = poseY;

      currentSmooth++;
      currentSmooth%=5;

      double poseX = 0.0;
      double poseY = 0.0;
      for (int i = 0; i < smoothX.length; i++) { poseX += smoothX[i]; }
      for (int i = 0; i < smoothY.length; i++) { poseY += smoothY[i]; }
      poseX /= 5;
      poseY /= 5;

      if (poseX-1.5 > 0.05) {
        x = 0.1;
      }
      if (poseY-5.5 > 0.05) {
        y = 0.1;
      }
      if (poseX-1.5 < -0.05) {
        x = -0.4;
      }
      if (poseY-5.5 < -0.05) {
        y = -0.1;
      }
    map.swerve.swerveDriveF(x, y, 0.0, false);
  }
} 