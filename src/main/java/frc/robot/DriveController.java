package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.LoosenClimber;
import frc.robot.subsystems.climber.TightenClimber;
import frc.robot.subsystems.swerve.SwerveTurnTo;

import java.util.Arrays;

/**
 * <p> This is the specific controller that controls Swerve due to the fact that swerve requires 2 separate joysticks and buttons to rezero the robot's gyro/position.
 * <p> This also contains controls for the climber since movement and the climber go hand in hand.
 * <p> This also contains Vision buttons (that currently do nothing) and a WestCoast control (that does not work, but was useful for testing with a different robot)
 */
public class DriveController extends CommandXboxController {

    private final RobotMap map;
    private final RobocketsShuffleboard shuffleboard;

    /**
     * Initializes the Shooter Controller and makes an internal copy of the RobotMap to save performance.
     * @param port The port of the controller on the Dashboard
     * @param map The currently used RobotMap
     * @param shuffleboard A copy of the shuffleboard for settings purposes
     */
    public DriveController(int port, RobotMap map, RobocketsShuffleboard shuffleboard) {
        super(port);
        this.map = map;
        this.shuffleboard = shuffleboard;

        // TODO: this does not really do anything
        a().toggleOnTrue(Commands.runOnce(() -> System.out.println(map.vision)));

        x().toggleOnTrue(Commands.runOnce(map.swerve::zeroGyro));
        y().toggleOnTrue(Commands.runOnce(map.swerve::resetPose));

        rightBumper().toggleOnTrue(
            Commands.startEnd(
                this::loosenClimber,
                this::tightenClimber,
                map.climber
            )
        );
    }

    private void loosenClimber() {
        CommandScheduler.getInstance().schedule(new LoosenClimber());
    }

    private void tightenClimber() {
        CommandScheduler.getInstance().schedule(new TightenClimber());
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
    // This records the past 20 inputs received from the controller, and averages them out
    // This way, rather than a controller going from 0 to 1 in 1 cycle, it takes a couple cycles to reach 1
    // This way, the motors to not instantly accelerate
    private final int SMOOTH_FRAME_LENGTH = 5;

    private int smoothNextFrameToWrite = 0;
    private final double[] smoothLeftX = new double[SMOOTH_FRAME_LENGTH];
    private final double[] smoothLeftY = new double[SMOOTH_FRAME_LENGTH];
    private final double[] smoothRightX = new double[SMOOTH_FRAME_LENGTH];

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
     * <p> This applies input smoothing to the joystick axex to make them smoother.
     * <p> This also checks for all button pushes and runs their respected Swerve commands/functions.
     */
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

        // Swerve
        double xyCof = 1;//0.75/Math.max(0.001, Math.sqrt(Math.pow(deadzone(controller.getLeftX(), 0.1), 2)+Math.pow(deadzone(controller.getLeftY(), 0.1), 2)));

        // ROBOT RELATIVE
        // map.swerve.swerveDriveR(new ChassisSpeeds(
        //     SmartDashboard.getNumber("Swerve Speed", 0.7) * -xyCof * deadzone(LeftY, 0.1),      // Forward/backwards
        //     SmartDashboard.getNumber("Swerve Speed", 0.7) * -xyCof * deadzone(LeftX, 0.1),    // Left/Right
        //     SmartDashboard.getNumber("Swerve Speed", 0.7) * deadzone(RightX, 0.08)   // Rotation
        // ));

        // FIELD RELATIVE
        if (RightX==0) {
            map.swerve.setDriveFXY(
                // On the controller, upwards is negative and left is also negative. To correct this, the negative version of both are sent.
                shuffleboard.getSettingNum("Movement Speed") * -xyCof * deadzone(LeftY, 0.1),      // Forward/backwards
                shuffleboard.getSettingNum("Movement Speed") * -xyCof * deadzone(LeftX, 0.1),    // Left/Right
                true); //square inputs to ease small adjustments
            map.swerve.setDriveRot(0, false);   // Should not be rotating if not rotating lol
        } else {
            map.swerve.swerveDriveF(
                // On the controller, upwards is negative and left is also negative. To correct this, the negative version of both are sent.
                shuffleboard.getSettingNum("Movement Speed") * -xyCof * deadzone(LeftY, 0.1),      // Forward/backwards
                shuffleboard.getSettingNum("Movement Speed") * -xyCof * deadzone(LeftX, 0.1),    // Left/Right
                shuffleboard.getSettingNum("Rotation Speed") * deadzone(RightX, 0.08),   // Rotation
                true); //square inputs to ease small adjustments
        }

        // turn to align with gyro
        if(getHID().getPOV() !=-1) {
            CommandScheduler.getInstance().schedule(new SwerveTurnTo(map.swerve, new Rotation2d(-getHID().getPOV() *0.01745329)));
        }

        // West Coast
        if (map.westcoast != null) {
            map.westcoast.arcadeDrive(getLeftY(), getRightX());
        }
    }


}
