package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveTurnTo;

// The reason for the existence of this is that it takes a TON of code out of Robot.java (that is all)
public class RobocketsController extends CommandXboxController {

    private RobotMap map;

    public RobocketsController(int port, RobotMap map) {
        super(port);
        this.map = map;
        // This is not a number I saw loaded anywhere
        SmartDashboard.putNumber("Swerve Speed",0.5);

        if (map.shooter != null) {
            a().onTrue(Commands.runOnce(this::onPressA, map.shooter));
            b().onTrue(Commands.runOnce(this::onPressB, map.shooter));
            a().onFalse(Commands.runOnce(this::onReleaseA, map.shooter));
            b().onFalse(Commands.runOnce(this::onReleaseB, map.shooter));
        }
        if (map.swerve != null) {
            x().onTrue(Commands.runOnce(this::onPressX, map.swerve));
            y().onTrue(Commands.runOnce(this::onPressY, map.swerve));
            x().onFalse(Commands.runOnce(this::onReleaseX, map.swerve));
            y().onFalse(Commands.runOnce(this::onReleaseX, map.swerve));
            leftTrigger().onTrue(Commands.runOnce(this::onLeftTrigger, map.swerve));
            rightTrigger().onTrue(Commands.runOnce(this::onRightTrigger, map.swerve));
        }
        if (map.intake != null) {
            leftBumper().onTrue(Commands.runOnce(this::onLeftBumper, map.intake));
            rightBumper().onTrue(Commands.runOnce(this::onRightBumper, map.intake));
        }
    }

    private void onPressA() {
        //CommandScheduler.getInstance().schedule(new Shoot(SmartDashboard.getNumber("Shooter Speed", 0.5)));
        map.shooter.setSpeed(SmartDashboard.getNumber("Shooter In Speed", 0.5));
    }

    private void onPressB() {
        //CommandScheduler.getInstance().schedule(new Shoot(-SmartDashboard.getNumber("Shooter Speed", 0.5)));
        map.shooter.setSpeed(-SmartDashboard.getNumber("Shooter Out Speed", 0.5));
    }

    private void onReleaseA() {
        map.shooter.setSpeed(0);
    }

    private void onReleaseB() {
        map.shooter.setSpeed(0);
    }

    private void onPressX() {
        map.shooter.setIntakeSpeed(-SmartDashboard.getNumber("Shooter Outtake Speed", 0.5));
    }

    private void onPressY() {
        map.shooter.setIntakeSpeed(SmartDashboard.getNumber("Shooter Intake Speed", 0.5));
    }

    private void onReleaseX() {
        map.shooter.setIntakeSpeed(0);
    }

    private void onReleaseY() {
        map.shooter.setIntakeSpeed(0);
    }

    private void onLeftBumper() {
        if (map.intake != null) {
            map.intake.intake();
        }
        map.leds.ChargeUpSeq();
        map.leds.NoteIndicator(true);
    }

    private void onRightBumper() {
        if (map.intake != null) {
            map.intake.outtake();
        }
        map.leds.NoteIndicator(false);
    }

    private void onLeftTrigger() {
        map.swerve.zeroGyro();
    }

    private void onRightTrigger() {
        map.swerve.resetPose();
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
    private final int SMOOTH_FRAME_LENGTH = 5;

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
                    SmartDashboard.getNumber("Swerve Speed", 0.7) * -xyCof * deadzone(LeftY, 0.1),      // Foward/backwards
                    SmartDashboard.getNumber("Swerve Speed", 0.7) * -xyCof * deadzone(LeftX, 0.1),    // Left/Right
                    true); //square inputs to ease small adjustments
                map.swerve.setDriveRot(0, false);   // Should not be rotating if not rotating lol
            } else {
                map.swerve.swerveDriveF(
                    // On the controller, upwards is negative and left is also negative. To correct this, the negative version of both are sent.
                    SmartDashboard.getNumber("Swerve Speed", 0.7) * -xyCof * deadzone(LeftY, 0.1),      // Foward/backwards
                    SmartDashboard.getNumber("Swerve Speed", 0.7) * -xyCof * deadzone(LeftX, 0.1),    // Left/Right
                    SmartDashboard.getNumber("Swerve Speed", 0.7) * deadzone(RightX, 0.08),   // Rotation
                    true); //square inputs to ease small adjustments
            }

            // turn to align with gyro
            if(getPOV()!=-1) {
                CommandScheduler.getInstance().schedule(new SwerveTurnTo(map.swerve, new Rotation2d(-getPOV()*0.01745329)));
            }
        }
        // Intake
        if (map.intake != null) {
            map.intake.rotate(deadzone(getRightX(), 0.1));
        }

        // West Coast
        if (map.westcoast != null) {
            map.westcoast.arcadeDrive(getLeftY(), getRightX());
        }
    }

    public int getPOV() {
        return getHID().getPOV();
    }
}
