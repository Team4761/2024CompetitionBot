package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.hardware.TalonFX;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    //TalonFX motorLeft;  // Assuming that the front of the shooter is the forward direction
    TalonFX motorRight; // Assuming that the front of the shooter is the forward direction

    public ShooterSubsystem() {
        //motorLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR_PORT);
        motorRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR_PORT);
    }

    public void setSpeed(double speed) {
        //motorLeft.set(TalonFXControlMode.PercentOutput, speed);
        //motorRight.set(TalonFXControlMode.PercentOutput,-speed);
        //motorLeft.set(speed);
        motorRight.set(-speed);
    }
}
