// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobocketsShuffleboard;

/**
 * This has been created by religiously following photonvisions docs
 */
public class VisionSubsystem2 extends SubsystemBase {

  // Basic variables
  PhotonCamera camera = new PhotonCamera("Camera");
  PhotonPipelineResult result;
  boolean hasTargets;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget target;

  // April Tag variables
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  //PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamera);

  // Transformation values

  // This is the relation between the camera and the robot
  // REMINDER: transform describes how to move from 1 pose to another, pose is the position and rotation of a desired location
  // TODO: get this value
  Translation3d cameraToRobotTranslation = new Translation3d(0.3915, 0, 0.35);
  Rotation3d cameraToRobotRotation = new Rotation3d(0,Math.PI/12.0,0);
  Transform3d cameraToRobot = new Transform3d(cameraToRobotTranslation, cameraToRobotRotation);

  Transform3d cameraSpaceTransform;
  Pose3d fieldSpaceTransform;

  /** Creates a new VisionSubsystem2. */
  public VisionSubsystem2() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    targets = result.getTargets();
    target = result.getBestTarget();

    cameraSpaceTransform = target.getBestCameraToTarget();
    fieldSpaceTransform = PhotonUtils.estimateFieldToRobotAprilTag(
      target.getBestCameraToTarget(), 
      aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), 
      cameraToRobot
    );

    double cameraSpaceTransformX = cameraSpaceTransform.getX();
    double cameraSpaceTransformY = cameraSpaceTransform.getY();
    RobocketsShuffleboard.addNumber("Camera Space X", cameraSpaceTransformX);
    RobocketsShuffleboard.addNumber("Camera Space Y", cameraSpaceTransformY);
  }

  /**
   * returns the distance in meters from the best camera target
   * @return eucliden distance to target
   */
  public double getAprilTagDistance() {
    if(!hasTargets) return -1.0;

    Transform3d t = target.getBestCameraToTarget();
    double x = t.getX();
    double y = t.getY();
    double z = t.getZ();
    double distanceToTarget = Math.sqrt(x*x + y*y + z*z);

    return distanceToTarget;
  }

  /**
   * returns the transform from the camera to the apriltag
   * used if we want to align to a special offset
   * @return transform3d from camera to target
   */
  public Transform3d getCameraToAprilTag() {
    if(!hasTargets) return null;

    return target.getBestCameraToTarget();
  }
}
