package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    // Neos that rotate the entire intake system (left or right facing forward)
    CANSparkMax intakeRotateL;
    CANSparkMax intakeRotateR;
    // Neos that actually intake
    CANSparkMax intakeL;
    CANSparkMax intakeR;

    private double intakeSpeed = 0.5;

    public IntakeSubsystem() {
        intakeRotateL = new CANSparkMax(Constants.INTAKE_LEFT_ROTATE_PORT, MotorType.kBrushless);
        intakeRotateR = new CANSparkMax(Constants.INTAKE_RIGHT_ROTATE_PORT, MotorType.kBrushless);
        intakeL = new CANSparkMax(Constants.INTAKE_LEFT_PORT, MotorType.kBrushless);
        intakeR = new CANSparkMax(Constants.INTAKE_RIGHT_PORT, MotorType.kBrushless);
    }

    // Speed is a value between -1 and 1 where -1 SHOULD BE down
    public void rotate(double speed) {
        intakeRotateL.set(speed);
        intakeRotateR.set(-speed);
    }

    public void intake() {
        intakeL.set(intakeSpeed);
        intakeR.set(-intakeSpeed);
    }

    public void outtake() {
        intakeL.set(-intakeSpeed);
        intakeR.set(intakeSpeed);
    }
}
