package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climber.LoosenClimber;
import frc.robot.subsystems.climber.TightenClimber;
import frc.robot.subsystems.swerve.SwerveTurnTo;

/**
 * <p> This is the specific controller that controls Swerve due to the fact that swerve requires 2 separate joysticks and buttons to rezero the robot's gyro/position.
 * <p> This also contains controls for the climber since movement and the climber go hand in hand.
 * <p> This also contains Vision buttons (that currently do nothing) and a WestCoast control (that does not work, but was useful for testing with a different robot)
 */
public class DriveController extends XboxController {

    private RobotMap map;
    private RobocketsShuffleboard shuffleboard;

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
    private final int SMOOTH_FRAME_LENGTH = 3;

    private int smoothNextFrameToWrite = 0;
    private double[] smoothLeftX = new double[SMOOTH_FRAME_LENGTH];
    private double[] smoothLeftY = new double[SMOOTH_FRAME_LENGTH];
    private double[] smoothRightX = new double[SMOOTH_FRAME_LENGTH];

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
        if (map.swerve != null) {
            double xyCof = 1;//0.75/Math.max(0.001, Math.sqrt(Math.pow(deadzone(controller.getLeftX(), 0.1), 2)+Math.pow(deadzone(controller.getLeftY(), 0.1), 2)));

            // ROBOT RELATIVE
            // map.swerve.swerveDriveR(new ChassisSpeeds(
            //     SmartDashboard.getNumber("Swerve Speed", 0.7) * -xyCof * deadzone(LeftY, 0.1),      // Foward/backwards
            //     SmartDashboard.getNumber("Swerve Speed", 0.7) * -xyCof * deadzone(LeftX, 0.1),    // Left/Right
            //     SmartDashboard.getNumber("Swerve Speed", 0.7) * deadzone(RightX, 0.08)   // Rotation
            // ));

            // FIELD RELATIVE
            if (RightX==0) {
                map.swerve.setDriveFXY(
                    // On the controller, upwards is negative and left is also negative. To correct this, the negative version of both are sent.
                    shuffleboard.getSettingNum("Movement Speed") * -xyCof * deadzone(LeftY, 0.1),      // Foward/backwards
                    shuffleboard.getSettingNum("Movement Speed") * -xyCof * deadzone(LeftX, 0.1),    // Left/Right
                    true); //square inputs to ease small adjustments
                map.swerve.setDriveRot(0, false);   // Should not be rotating if not rotating lol
            } else {
                map.swerve.swerveDriveF(
                    // On the controller, upwards is negative and left is also negative. To correct this, the negative version of both are sent.
                    shuffleboard.getSettingNum("Movement Speed") * -xyCof * deadzone(LeftY, 0.1),      // Foward/backwards
                    shuffleboard.getSettingNum("Movement Speed") * -xyCof * deadzone(LeftX, 0.1),    // Left/Right
                    shuffleboard.getSettingNum("Rotation Speed") * deadzone(RightX, 0.08),   // Rotation
                    true); //square inputs to ease small adjustments
            }

            if (getXButtonPressed()) {
                map.swerve.zeroGyro();
            }
            if (getYButtonPressed()) {
                map.swerve.resetPose();
            }

            // turn to align with gyro
            if(getPOV()!=-1) {
                CommandScheduler.getInstance().schedule(new SwerveTurnTo(map.swerve, new Rotation2d(-getPOV()*0.01745329)));
            }
        }

        // Climber
        if (map.climber != null) {
            if (getRightBumperPressed()) {
                CommandScheduler.getInstance().schedule(new LoosenClimber());
            }
            if (getRightBumperReleased()) {
                CommandScheduler.getInstance().schedule(new TightenClimber());
            }
        }

        // Intake
        if (map.intake != null) {

            if (getLeftBumperPressed()) {
                map.intake.intake(shuffleboard.getSettingNum("Intake Speed"));
            }
            else if (getRightBumperPressed()) {
                map.intake.outtake(shuffleboard.getSettingNum("Outtake Speed"));
            }   

            else if(getRightBumperReleased() || getLeftBumperReleased()){
                map.intake.stop();
            }

            else if (getAButtonPressed()){
                map.intake.goToRotation(new Rotation2d(Constants.INTAKE_START_POSITION));    // Starting position (upright)
            }
            else if (getBButtonPressed()){
                map.intake.goToRotation(new Rotation2d(Constants.INTAKE_INTAKE_POSITION)); // Actual intake position (on the ground)
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
