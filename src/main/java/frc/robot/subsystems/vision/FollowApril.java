// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class FollowApril extends Command {

  RobotMap map = Robot.getMap();

  private double mdistance = -1.0;
  private Pose2d offset = null;

  private String poseMode;
  private final String distanceMode = "Distance";
  private final String offsetMode = "Offset";

  // I don't think we'll need to use this distance mode, but I'm keeping it just incase
  /**
   * Creates a new FollowApril Command
   * This command is used to follow an april tag with a given desired offset from the target
   * @param mdistance euclidean distance from best target in meters
   */
  /*public FollowApril(double mdistance) {
    addRequirements(map.vision);
    addRequirements(map.swerve);
    this.mdistance = mdistance;
    // Use addRequirements() here to declare subsystem dependencies.
  }*/

  /**
   * Creates a new FollowApril Command
   * This command is used to follow an april tag with a given desired offset from the target
   * @param offset Pose2d offset from target. Translation in meters, rotation in radians
   */
  public FollowApril(Pose2d offset) {
    addRequirements(map.vision2);
    addRequirements(map.swerve);
    this.offset = offset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mdistance != -1) poseMode = distanceMode;
    if(offset != null) poseMode = offsetMode;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(poseMode == distanceMode) executeDistanceMode();
    else executeOffsetMode();
  }

  private void executeDistanceMode() {
    map.vision2.getAprilTagDistance();

  }

  private void executeOffsetMode() {

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
