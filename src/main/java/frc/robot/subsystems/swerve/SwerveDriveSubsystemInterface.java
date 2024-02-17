package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveDriveSubsystemInterface extends Subsystem {
    void configureAutoBuilder();

    // set how fast the swerve drive turns, rad/s allegedly
    void setDriveRot(double sR, boolean squareInputs);

    // set how fast the swerve drive goes, +x is forwards, +y is left and m/s hopefully
    void setDriveFXY(double sX, double sY, boolean squareInputs);

    // Field Oriented swerve drive, m/s, m/s, rad/s or something, +x is forwards, +y is left
    void swerveDriveF(double sX, double sY, double sR, boolean squareInputs);

    // Robot oriented swerve drive, m/s, m/s, rad/s
    // +speed is forwards, +strafe is left
    void swerveDriveR(double speed, double strafe, double speedRot);

    void swerveDriveR(ChassisSpeeds newTargetStates);

    boolean isOnRedAlliance();

    // car, m/s, degrees
    void carDrive(double speed, double turn);

    void setTargetAngle(Rotation2d r);

    // stuff
    // Degrees
    double getGyroDegrees();

    // Radians
    Rotation2d getGyroRotation();

    Pose2d getPose();

    //for on the go field oriented and stuff
    void zeroGyro();

    // Reset the expected position of the bot
    void resetPose();

    // Reset the inputted pose
    // Only used by the holonomic builder
    void resetPose(Pose2d pose2d);

    // This is mainly just for testing. This is what the wiki says to do.
    Pose2d getPoseForPathPlanner();

    // This is mainly just for testing. This is what the wiki says to do.
    void resetPoseForPathPlanner(Pose2d pose);

    // Returns the current robot-relative chasis speeds.
    ChassisSpeeds getRobotRelativeSpeeds();

    void stop();

    void test();
}
