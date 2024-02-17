package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystemMock extends SubsystemBase implements ShooterSubsystemInterface {
    @Override
    public void getShooterToSetSpeed() {
    }

    @Override
    public void getShooterToSetAngle() {

    }

    @Override
    public void setShooterSpeed(double speed) {

    }

    @Override
    public void setShooterAngle(double angleRadians) {

    }

    @Override
    public void rotate(double angleRadians) {

    }

    @Override
    public void setIntakeSpeed(double speed) {

    }

    @Override
    public double getShooterAngle() {
        return 0;
    }

    @Override
    public double getShooterAngleVelocity() {
        return 0;
    }

    @Override
    public boolean isPieceInUpperIntake() {
        return false;
    }

    @Override
    public boolean isPieceInLowerIntake() {
        return false;
    }
}
