package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    TalonFX motorLeft;  // Assuming that the front of the shooter is the forward direction
    TalonFX motorRight; // Assuming that the front of the shooter is the forward direction
    CANSparkMax intakeLeft;     // Assuming that the front of the shooter is the forward direction
    CANSparkMax intakeRight;    // Assuming that the front of the shooter is the forward direction

    public ShooterSubsystem() {
        motorLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR_PORT);
        motorRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR_PORT);
        intakeLeft = new CANSparkMax(Constants.SHOOTER_INTAKE_LEFT_MOTOR_PORT, MotorType.kBrushless);
        intakeRight = new CANSparkMax(Constants.SHOOTER_INTAKE_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    }

    // Sets the SHOOTING speed
    public void setSpeed(double speed) {
        motorLeft.set(speed);
        motorRight.set(speed);  // As of Jan 20, 2024, the speeds are not reversed
    }

    // Sets the speed of the intake to the motor's speed. (the belt that pulls the game piece from the actual intake)
    public void setIntakeSpeed(double speed) {
        intakeLeft.set(speed);
        intakeRight.set(-speed);
    }
}
