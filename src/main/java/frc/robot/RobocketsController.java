package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.Shoot;

// The reason for the existence of this is that it takes a TON of code out of Robot.java (that is all)
public class RobocketsController extends XboxController {
    
    private RobotMap map;

    public RobocketsController(int port, RobotMap map) {
        super(port);
        this.map = map;
        // This is not a number I saw loaded anywhere
        SmartDashboard.putNumber("Swerve Speed",0.5);
    }

    // Apply a deadzone for swerve
    public static double deadzone (double value, double deadzone) {
        if (Math.abs(value) > deadzone) {
            if (value > 0.0) { return (value - deadzone) / (1.0 - deadzone); } 
            else             { return (value + deadzone) / (1.0 - deadzone); }
        }
        return 0.0;
    }

    // Apply input smoothing
    // This records the past 20 inputs received from the controller, and averages them out
    // This way, rather than a controller going from 0 to 1 in 1 cycle, it takes a couple cycles to reach 1
    // This way, the motors to not instantly accelerate
    private final int SMOOTH_FRAME_LENGTH = 20;

    private int smoothNextFrameToWrite = 0;
    private double[] smoothLeftX = new double[SMOOTH_FRAME_LENGTH];
    private double[] smoothLeftY = new double[SMOOTH_FRAME_LENGTH];
    private double[] smoothRightX = new double[SMOOTH_FRAME_LENGTH];

    private double smooth(double[] history) {
        double average = 0;
        for(int i = 0; i < history.length; i++) {
            average += history[i];
        }
        average /= (double)history.length;
        return average;
    }

    public void teleopPeriodic() {

        // smooth out the xbox inputs
        smoothLeftX[smoothNextFrameToWrite] = getLeftX();
        smoothLeftY[smoothNextFrameToWrite] = getLeftY();
        smoothRightX[smoothNextFrameToWrite] = getRightX();
        smoothNextFrameToWrite++;
        smoothNextFrameToWrite %= SMOOTH_FRAME_LENGTH;

        double LeftX = smooth(smoothLeftX);
        double LeftY = smooth(smoothLeftY);
        double RightX = smooth(smoothRightX);

        // double LeftX = getLeftX();
        // double LeftY = getLeftY();
        // double RightX = getRightX();

        // Swerve
        if (map.swerve != null) {
            double xyCof = 1;//0.75/Math.max(0.001, Math.sqrt(Math.pow(deadzone(controller.getLeftX(), 0.1), 2)+Math.pow(deadzone(controller.getLeftY(), 0.1), 2)));
            map.swerve.swerveDriveF(
                    // The robot is labeled slightly improperly in relation to the gyro, so the X and Y axis are flipped.
                    SmartDashboard.getNumber("Swerve Speed", 0.5) * -xyCof * deadzone(LeftX, 0.1)/* * (controller.getLeftTriggerAxis()+controller.getRightTriggerAxis())*/,      // Foward/backwards
                    SmartDashboard.getNumber("Swerve Speed", 0.5) * xyCof * deadzone(LeftY, 0.1)/*  * (controller.getLeftTriggerAxis()+controller.getRightTriggerAxis())*/,    // Left/Right
                    SmartDashboard.getNumber("Swerve Speed", 0.5) * deadzone(RightX, 0.08));   // Rotation
            
            if(getXButtonPressed()) {
                map.swerve.zeroGyro();
            }
            if(getYButtonPressed()) {
                map.swerve.resetPose(); 
            }
        }
        // Intake
        // if (map.intake != null) {
        //     map.intake.rotate(deadzone(getRightX(), 0.1));

        //     if (getLeftBumperPressed()) {
        //         map.intake.intake();
        //     }
        //     else if (getRightBumperPressed()) {
        //         map.intake.outtake();
        //     }
        //     }
        // Shooter
        if (map.shooter != null) {
            if (getAButtonPressed()) {
                //CommandScheduler.getInstance().schedule(new Shoot(SmartDashboard.getNumber("Shooter Speed", 0.5)));
                map.shooter.setSpeed(SmartDashboard.getNumber("Shooter In Speed", 0.5));
            }
            if (getBButtonPressed()) {
                //CommandScheduler.getInstance().schedule(new Shoot(-SmartDashboard.getNumber("Shooter Speed", 0.5)));
                map.shooter.setSpeed(-SmartDashboard.getNumber("Shooter Out Speed", 0.5));
            }
            if (getYButtonPressed()) {
                map.shooter.setIntakeSpeed(SmartDashboard.getNumber("Shooter Intake Speed", 0.5));
            }
            if (getXButtonPressed()) {
                map.shooter.setIntakeSpeed(-SmartDashboard.getNumber("Shooter Outtake Speed", 0.5));
            }
            if (getAButtonReleased() || getBButtonReleased()) {
                map.shooter.setSpeed(0);
            }
            if (getXButtonReleased() || getYButtonReleased()) {
                map.shooter.setIntakeSpeed(0);
            }
        }
        // Vision
        if (map.vision != null) {
            if(getAButtonPressed()){
                map.vision.toString();
            }
        }



        // West Coast
        if (map.westcoast != null) {
            map.westcoast.arcadeDrive(getLeftY(), getRightX());
        }
    }
}
