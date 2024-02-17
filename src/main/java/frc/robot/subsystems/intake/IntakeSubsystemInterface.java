package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeSubsystemInterface extends Subsystem {
    void getIntakeToSetAngle();

    void intake(double speed);

    void outtake(double speed);

    void rotate(double offsetRadians);

    void goToRotation(Rotation2d rotation);

    Rotation2d getIntakeAngle();

    double getIntakeAngleVelocity();
}
