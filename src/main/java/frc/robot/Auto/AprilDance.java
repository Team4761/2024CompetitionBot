// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * <p> Hey Marcus this is your job!
 * 
 * <p> This class reads an april tag from the camera, and then positions the robot infront of the april tag
 * <p> TODO:
 * <p> + How do you change the pose2d from the april tag?
 * <p> + How do you deal with a rotated april tag?
 * <p> + What do you do if there are multiple tags?
 * <p> + What do you do if an april tag is no longer detectable?
 * <p> + When should this command end?
 */
public class AprilDance extends Command {

  private double m_distanceFromTag = 0.0;

  /**
   * Creates an AprilDance command
   * @param swerve swerve subsystem to control the wheels
   * @param vision vision subsystem to supply visual
   * @param distanceFromTag distance from the tag in meters
   */
  public AprilDance(SwerveDriveSubsystem swerve, VisionSubsystem vision, double distanceFromTag) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    addRequirements(vision);
    m_distanceFromTag = distanceFromTag;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
