// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class FollowApril extends Command {

  RobotMap map = Robot.getMap();

  private Pose3d targetRobotToTag = null;

  /**
   * Creates a new FollowApril Command
   * This command is used to follow an april tag with a given desired offset from the target
   * @param offset Pose2d offset from target. Translation in meters, rotation in radians
   */
  public FollowApril(Pose3d offset) {
    addRequirements(map.vision2);
    addRequirements(map.swerve);
    this.targetRobotToTag = offset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // r-C + C-T = R-T
    Transform3d robotToTag = map.vision2.getCameraToAprilTag().plus(map.vision2.cameraToRobot.inverse());

    // we cannot change the rotation so instead we will try to align ourselves with the translation
    Translation3d robotToTagTransform = robotToTag.getTranslation();

    // get the delta
    Translation3d robotToTagDelta = targetRobotToTag.getTranslation().minus(robotToTagTransform);

    // get the x and y
    double DeltaX = robotToTagDelta.getX();
    double DeltaY = robotToTagDelta.getY();

    // send these values to the swervedrive
    double magicmultiplier = 0.2;
    map.swerve.swerveDriveR(DeltaX*magicmultiplier,DeltaY*magicmultiplier,0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
