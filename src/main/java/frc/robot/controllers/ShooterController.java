package frc.robot.controllers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobocketsShuffleboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.shooter.AutoShooterIntake;
import frc.robot.subsystems.shooter.AutoSourceIntake;
import frc.robot.subsystems.shooter.IntakeAndShoot;

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
    private final int SMOOTH_FRAME_LENGTH = 1;

    private int smoothNextFrameToWrite = 0;
    private double[] smoothLeftY = new double[SMOOTH_FRAME_LENGTH];     // Contains the past {SMOOTH_FRAME_LENGTH} number of inputs.
    private double[] smoothRightY = new double[SMOOTH_FRAME_LENGTH];    // Contains the past {SMOOTH_FRAME_LENGTH} number of inputs.

    /**
     * <p> Applies input smoothing.
     * <p> This averages the values of the input array of doubles to smooth out the transitions between inputs.
     * @param history A list of the last {SMOOTH_FRAME_LENGTH} number of axis inputs. The longer the list, the smoother it is but also the more input delay there is.
     * @return The average (mean) value of the input list of doubles. This will be the average value of a joystick axis.
     */
    private double smooth(double[] history) {
        double average = 0;
        for(int i = 0; i < history.length; i++) {
            average += history[i];
        }
        average /= (double)history.length;
        return average;
    }

    /**
     * <p> This should run during the Robot.java's teleopPeriodic method.
     * <p> This applies input smoothing to the joystick axises to make them smoother.
     * <p> This also checks for all button pushes and runs their respected Shooter and Intake commands.
     */
    public void teleopPeriodic() {
        smoothLeftY[smoothNextFrameToWrite] = deadzone(getLeftY(), 0.08);
        smoothRightY[smoothNextFrameToWrite] = deadzone(getRightY(), 0.15);
        smoothNextFrameToWrite++;
        smoothNextFrameToWrite %= SMOOTH_FRAME_LENGTH;

        double LeftY = smooth(smoothLeftY);
        double RightY = smooth(smoothRightY);

               

        // Shooter
        if (map.shooter != null) {

            map.shooter.rotate(-0.04*deadzone(getLeftY(), 0.08)); // sets target pos

            if (getAButtonPressed()) { //shoot
                CommandScheduler.getInstance().schedule(new IntakeAndShoot(shuffleboard.getSettingNum("Shooter Out Speed")));
            }
            
            if (getPOV()==0) { // shoot at amp speed
                CommandScheduler.getInstance().schedule(new IntakeAndShoot(10l));
            }
            if (getPOV() == 180) { // down dpad for source intake
                map.shooter.setShooterSpeed(-shuffleboard.getSettingNum("Shooter In Speed"));
                // CommandScheduler.getInstance().schedule(new AutoSourceIntake());
            }

            if (getYButtonPressed()) { // uptakes until top breakbeam
                CommandScheduler.getInstance().schedule(new AutoShooterIntake());
            }
            
            if (getLeftBumperPressed()) {
                map.shooter.setShooterAngle(Units.degreesToRadians(47));    // ground intake angle
            }
            if (getRightBumperPressed()) {
                map.shooter.setShooterAngle(Units.degreesToRadians(60));    // shooting/amp/source intake angle
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

        // Intake
        if (map.intake != null) {
            // map.intake.rotate(getRightY());
            map.intake.setAngleMotorSpeed(-deadzone(getRightY(), 0.15)*0.3);
            
            if (getBButtonPressed()) {
                map.intake.intake(Robot.getShuffleboard().getSettingNum("Intake Speed")); // goes up
            }
            if (getXButtonPressed()) {
                map.intake.outtake(Robot.getShuffleboard().getSettingNum("Outtake Speed")); // goes down
            }
            if (getXButtonReleased() || getBButtonReleased()) {
                map.intake.intake(0);
            }
        }
    }
}
