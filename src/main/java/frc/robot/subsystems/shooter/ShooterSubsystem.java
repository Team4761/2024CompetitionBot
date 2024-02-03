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

    private double targetSpeed;

    public ShooterSubsystem() {
        motorLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR_PORT);
        motorRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR_PORT);
        intakeLeft = new CANSparkMax(Constants.SHOOTER_INTAKE_LEFT_MOTOR_PORT, MotorType.kBrushless);
        intakeRight = new CANSparkMax(Constants.SHOOTER_INTAKE_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    }

    public void periodic() {
        double avgEncoderSpd = (motorLeft.getVelocity().getValueAsDouble()+motorRight.getVelocity().getValueAsDouble())/2; //rotations per second allegedly

        double accelFactor = 0.005 * (targetSpeed-avgEncoderSpd); // P kinda
        double feedForwardV = 0.1 * targetSpeed; //magic numbers no math or testing done yet, ideally sets the motors to near the velocity
        double feedForwardS = 0; //whatever number to maintain the speed
        
        double spdOut = accelFactor + feedForwardV + feedForwardS;

        motorLeft.set(spdOut); //probably test then use setVoltage
        motorRight.set(spdOut);  // As of Jan 20, 2024, the speeds are not reversed
    }

    // Sets the SHOOTING speed in rotations per second (hopefully), does not change until stated otherwise
    public void setSpeed(double speed) {
        targetSpeed = speed;
    }

    // Sets the speed of the intake to the motor's speed. (the belt that pulls the game piece from the actual intake)
    public void setIntakeSpeed(double speed) {
        intakeLeft.set(speed);
        intakeRight.set(-speed);
    }
}
