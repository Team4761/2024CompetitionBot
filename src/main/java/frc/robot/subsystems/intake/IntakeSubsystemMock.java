package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystemMock extends SubsystemBase implements IntakeSubsystemInterface {
    Rotation2d rotate2d = new Rotation2d();
    @Override
    public void getIntakeToSetAngle() {
    }

    @Override
    public void intake(double speed) {

    }

    @Override
    public void outtake(double speed) {

    }

    @Override
    public void rotate(double offsetRadians) {

    }

    @Override
    public void goToRotation(Rotation2d rotation) {
        rotate2d = rotation;
    }

    @Override
    public Rotation2d getIntakeAngle() {
        return rotate2d;
    }

    @Override
    public double getIntakeAngleVelocity() {
        return 0;
    }
}
