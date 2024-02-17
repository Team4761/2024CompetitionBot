package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Arrays;

/**
 * This is the code for the controller that controls the shooter and the intake.
 */
public class ShooterController extends CommandXboxController {
    
    private final RobotMap map;
    private final RobocketsShuffleboard shuffleboard;

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

        a().toggleOnTrue(
            Commands.startEnd(
                this::shooterIn,
                this::stopShooter,
                map.shooter
            )
        );
        b().toggleOnTrue(
            Commands.startEnd(
                this::shooterOut,
                this::stopShooter,
                map.shooter
            )
        );
        x().toggleOnTrue(
            Commands.startEnd(
                this::shooterIntakeIn,
                this::stopShooterIntake,
                map.shooter
            )
        );
        y().toggleOnTrue(
            Commands.startEnd(
                this::shooterIntakeOut,
                this::stopShooterIntake,
                map.shooter
            )
        );

        leftBumper().toggleOnTrue(
            Commands.startEnd(
                this::intakeOn,
                this::intakeOff,
                map.intake, map.leds
            )
        );
        rightBumper().toggleOnTrue(
            Commands.startEnd(
                this::outtakeOn,
                this::outtakeOff,
                map.intake, map.leds
            )
        );
    }

    private void shooterIn() {
        map.shooter.setShooterSpeed(shuffleboard.getSettingNum("Shooter In Speed"));
    }

    private void shooterOut() {
        map.shooter.setShooterSpeed(-shuffleboard.getSettingNum("Shooter Out Speed"));
    }

    private void stopShooter() {
        map.shooter.setShooterSpeed(0);
    }

    private void shooterIntakeIn() {
        map.shooter.setIntakeSpeed(shuffleboard.getSettingNum("Shooter Intake Speed"));
    }

    private void shooterIntakeOut() {
        map.shooter.setIntakeSpeed(-shuffleboard.getSettingNum("Shooter Outtake Speed"));
    }

    private void stopShooterIntake() {
        map.shooter.setIntakeSpeed(0);
    }

    private void intakeOn() {
        map.intake.intake(shuffleboard.getSettingNum("Intake Speed"));
        map.leds.NoteIndicator(true);
    }

    private void intakeOff() {
        map.intake.intake(0);
    }

    private void outtakeOn() {
        map.intake.outtake(shuffleboard.getSettingNum("Outtake Speed"));
        map.leds.NoteIndicator(false);
    }

    private void outtakeOff() {
        map.intake.outtake(0);
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
    private final double[] smoothLeftY = new double[SMOOTH_FRAME_LENGTH];     // Contains the past {SMOOTH_FRAME_LENGTH} number of inputs.
    private final double[] smoothRightY = new double[SMOOTH_FRAME_LENGTH];    // Contains the past {SMOOTH_FRAME_LENGTH} number of inputs.

    /**
     * <p> Applies input smoothing.
     * <p> This averages the values of the input array of doubles to smooth out the transitions between inputs.
     * @param history A list of the last {SMOOTH_FRAME_LENGTH} number of axis inputs. The longer the list, the smoother it is but also the more input delay there is.
     * @return The average (mean) value of the input list of doubles. This will be the average value of a joystick axis.
     */
    private double smooth(double[] history) {
        return Arrays.stream(history).average().orElse(0.0);
    }

    /**
     * <p> This should run during the Robot.java's teleopPeriodic method.
     * <p> This applies input smoothing to the joystick axises to make them smoother.
     * <p> This also checks for all button pushes and runs their respected Shooter and Intake commands.
     */
    public void teleopPeriodic() {
        smoothLeftY[smoothNextFrameToWrite] = deadzone(getLeftY(), 0.08);
        smoothRightY[smoothNextFrameToWrite] = deadzone(getRightY(), 0.08);
        smoothNextFrameToWrite++;
        smoothNextFrameToWrite %= SMOOTH_FRAME_LENGTH;

        double LeftY = smooth(smoothLeftY);
        double RightY = smooth(smoothRightY);

        // Intake
        map.intake.rotate(RightY);

        // Shooter
        map.shooter.rotate(LeftY);
    }
}
