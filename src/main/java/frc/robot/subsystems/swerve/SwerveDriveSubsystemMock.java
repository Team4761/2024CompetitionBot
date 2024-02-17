package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystemMock extends SubsystemBase implements SwerveDriveSubsystemInterface {
    @Override
    public void configureAutoBuilder() {

    }

    @Override
    public void setDriveRot(double sR, boolean squareInputs) {

    }

    @Override
    public void setDriveFXY(double sX, double sY, boolean squareInputs) {

    }

    @Override
    public void swerveDriveF(double sX, double sY, double sR, boolean squareInputs) {

    }

    @Override
    public void swerveDriveR(double speed, double strafe, double speedRot) {

    }

    @Override
    public void swerveDriveR(ChassisSpeeds newTargetStates) {

    }

    @Override
    public boolean isOnRedAlliance() {
        return false;
    }

    @Override
    public void carDrive(double speed, double turn) {

    }

    @Override
    public void setTargetAngle(Rotation2d r) {

    }

    @Override
    public double getGyroDegrees() {
        return 0;
    }

    @Override
    public Rotation2d getGyroRotation() {
        return null;
    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public void zeroGyro() {

    }

    @Override
    public void resetPose() {

    }

    @Override
    public void resetPose(Pose2d pose2d) {

    }

    @Override
    public Pose2d getPoseForPathPlanner() {
        return null;
    }

    @Override
    public void resetPoseForPathPlanner(Pose2d pose) {

    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return null;
    }

    @Override
    public void stop() {

    }

    @Override
    public void test() {

    }
}
