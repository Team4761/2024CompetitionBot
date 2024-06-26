// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auto.AutoConstruct;
import frc.robot.subsystems.leds.FlowLEDs;
import frc.robot.subsystems.shooter.StopShooter;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.ShooterController;  

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Represents all physical objects on our robot
  // This is also the file where all the subsystems reside
  private static RobotMap map = new RobotMap();
  public static RobotMap getMap() { return map; }

  public static RobocketsShuffleboard shuffleboard = new RobocketsShuffleboard();
  public static RobocketsShuffleboard getShuffleboard() { return shuffleboard; }

  public static DriveController driveController = new DriveController(Constants.DRIVE_CONTROLLER_PORT, map, shuffleboard);
  public static ShooterController shooterController = new ShooterController(Constants.SHOOTER_CONTROLLER_PORT, map, shuffleboard);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    boolean win = true; // This is the most important line. DO NOT REMOVE.
    System.out.println("Status on winning: " + win);
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

  @Override
  public void autonomousInit() {
    //if (map.shooter != null)
      //map.shooter.setShooterAngle(map.shooter.getShooterAngle().getRadians());
    AutoConstruct.scheduleSelectedCommand(map);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //if (map.vision.hasTargets()) {
    //  map.vision.dance();
    //}
    //map.swerve.test();

    CommandScheduler.getInstance().run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {  
    CommandScheduler.getInstance().cancelAll();

    if (map.shooter != null) {
      map.shooter.setShooterAngleSpeed(0);
      map.shooter.setShooterAngle(map.shooter.getShooterAngle().getRadians());
      CommandScheduler.getInstance().schedule(new StopShooter()); // stops shooter with a deccel limit
    }
    if (map.autoEndingAngle != null && map.swerve != null) {
      map.swerve.zeroGyro(map.autoEndingAngle);
    }
    if (map.leds != null)
      // map.leds.SetAllColor(255,0,0);
      CommandScheduler.getInstance().schedule(new FlowLEDs());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    driveController.teleopPeriodic();
    shooterController.teleopPeriodic();
    // Run any commands
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    map.swerve.test();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
