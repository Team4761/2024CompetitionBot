// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
public class VisionSubSystem extends SubsystemBase {

  //**ALL ANGLES IN RADIANS ALL DISTANCES IN METERS**//
  //PhotonCamera mCamera;
  private static final VisionSubSystem INSTANCE = new VisionSubSystem();
  double mCameraHeight = .19;
  double mCameraPitch = Units.degreesToRadians(30);

  private boolean mDriverMode = false;
  public VisionSubSystem(){
    //Replace with name of cam
    // mCamera = new PhotonCamera("Main");
  }

  @Override
  public void periodic() {
    
  }

  public static VisionSubSystem getInstance(){
    return INSTANCE;
  }

  // public int getBestTagID(){
  //   return mCamera.getLatestResult().getBestTarget().getFiducialId();
  // }

  // public boolean hasTargets(){
  //   return mCamera.getLatestResult().hasTargets();
  // }
  // public Translation2d getTransDiff(double targetHeight){
  //   if(hasTargets()){
  //     double pitch = Units.degreesToRadians(mCamera.getLatestResult().getBestTarget().getPitch());
  //     double distance = PhotonUtils.calculateDistanceToTargetMeters(
  //       mCameraHeight, 
  //       targetHeight, 
  //       mCameraPitch,
  //       pitch);
  //     double yaw = Units.degreesToRadians(mCamera.getLatestResult().getBestTarget().getYaw());
  //     double yDiff = distance * Math.cos(pitch);
  //     double xDiff = yDiff / Math.cos(yaw) * Math.sin(yaw);
  //     return new Translation2d(xDiff, yDiff - 1);
  //   }
  //   else{
  //     return new Translation2d();
  //   }
  // }

  // public void toggleDriverMode(){
  //   mDriverMode = !mDriverMode;
  //   mCamera.setDriverMode(mDriverMode);
  // }
  // //Returns false if not in driver mode
  // public boolean getDriverMode(){
  //   return mCamera.getDriverMode();
  // }
} 