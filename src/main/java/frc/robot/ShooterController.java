package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.Shoot;

/**
 * This is the code for the controller that controls the shooter and the intake.
 */
public class ShooterController extends XboxController {
    
    private RobotMap map;
    private RobocketsShuffleboard shuffleboard;

    /**
     * Initializes the Shooter Controller and makes an internal copy of the RobotMap to save performance.
     * @param port The port of the controller on the Dashboard
     * @param map The currently used RobotMap
     * @param shuffleboard A copy of the shuffleboard for settings purposes
     */
    public ShooterController(int port, RobotMap map, RobocketsShuffleboard shuffleboard) {
        super(port);
        this.map = map;
        this.shuffleboard = shuffleboard;
    }

    /**
     * Apply a deadzone to a given value to account for annoying controllers.
     * @param value The exact value that the controller reads (between -1.0 and 1.0)
     * @param deadzone The deadzone to be applied where anything below the deadzone is set to 0
     * @return The value that the controller reads AFTER the deadzone is applied.
     */
    public static double deadzone (double value, double deadzone) {
        if (Math.abs(value) > deadzone) {
            if (value > 0.0) { return (value - deadzone) / (1.0 - deadzone); } 
            else             { return (value + deadzone) / (1.0 - deadzone); }
        }
        return 0.0;
    }

    // Apply input smoothing
    // This records the past 5 inputs received from the controller, and averages them out
    // This way, rather than a controller going from 0 to 1 in 1 cycle, it takes a couple cycles to reach 1
    // This way, the motors to not instantly accelerate
    private final int SMOOTH_FRAME_LENGTH = 5;

    private int smoothNextFrameToWrite = 0;
    private double[] smoothLeftY = new double[SMOOTH_FRAME_LENGTH];
    private double[] smoothRightY = new double[SMOOTH_FRAME_LENGTH];

    private double smooth(double[] history) {
        double average = 0;
        for(int i = 0; i < history.length; i++) {
            average += history[i];
        }
        average /= (double)history.length;
        return average;
    }

    public void teleopPeriodic() {

        smoothLeftY[smoothNextFrameToWrite] = getLeftY();
        smoothRightY[smoothNextFrameToWrite] = getRightY();
        smoothNextFrameToWrite++;
        smoothNextFrameToWrite %= SMOOTH_FRAME_LENGTH;

        double LeftY = smooth(smoothLeftY);
        double RightY = smooth(smoothRightY);

        // Intake
        if (map.intake != null) {
            map.intake.rotate(deadzone(getRightX(), 0.1));

            if (getLeftBumperPressed()) {
                map.intake.intake();
            }
            else if (getRightBumperPressed()) {
                map.intake.outtake();
            }
        }

        // Shooter
        if (map.shooter != null) {
            if (getAButtonPressed()) {
                //CommandScheduler.getInstance().schedule(new Shoot(SmartDashboard.getNumber("Shooter Speed", 0.5)));
                map.shooter.setShooterSpeed(SmartDashboard.getNumber("Shooter In Speed", 0.5));
            }
            if (getBButtonPressed()) {
                //CommandScheduler.getInstance().schedule(new Shoot(-SmartDashboard.getNumber("Shooter Speed", 0.5)));
                map.shooter.setShooterSpeed(-SmartDashboard.getNumber("Shooter Out Speed", 0.5));
            }
            if (getYButtonPressed()) {
                map.shooter.setIntakeSpeed(SmartDashboard.getNumber("Shooter Intake Speed", 0.5));
            }
            if (getXButtonPressed()) {
                map.shooter.setIntakeSpeed(-SmartDashboard.getNumber("Shooter Outtake Speed", 0.5));
            }
            if (getAButtonReleased() || getBButtonReleased()) {
                map.shooter.setShooterSpeed(0);
            }
            if (getXButtonReleased() || getYButtonReleased()) {
                map.shooter.setIntakeSpeed(0);
            }
        }
        
        //LEDs
        if (map.leds != null){
            //if the intake button is pressed it will turn the LEds to orange
            if(getLeftBumperPressed())
            {
                map.leds.NoteIndicator(true);
            }

            if(getLeftBumperReleased())
            {
                map.leds.NoteIndicator(true);
            }
            //if the outake button is pressed it will turn the LEDs off
            if(getRightBumperPressed())
            {
                map.leds.NoteIndicator(false);
            }
            //if the outake button is pressed it will turn the LEDs off
            if(getAButtonPressed())
            {
                map.leds.NoteIndicator(false);
            }
        }
    }
}
