package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterSubsystemInterface extends Subsystem {
    void getShooterToSetSpeed();

    void getShooterToSetAngle();

    void setShooterSpeed(double speed);

    void setShooterAngle(double angleRadians);

    void rotate(double angleRadians);

    void setIntakeSpeed(double speed);

    double getShooterAngle();

    double getShooterAngleVelocity();

    boolean isPieceInUpperIntake();

    boolean isPieceInLowerIntake();
}
