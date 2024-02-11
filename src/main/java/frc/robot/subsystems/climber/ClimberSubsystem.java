package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

/**
 * <p> This should be the simplest subsystem as all it does is run a motor full force in one direction or let it go slack.
 * <p> NOTE: The motor that is connected MUST NOT BE on break mode.
 */
public class ClimberSubsystem {
    private CANSparkMax motor;  // The motor connected to the string inside the climber
    private double speed;       // The speed which the motor is constantly set to.

    /**
     * <p> Initializes the motor. That is all.
     */
    public ClimberSubsystem() {
        motor = new CANSparkMax(Constants.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
    }

    /**
     * The motor will constantly be set to {speed} speed which is a value between -1.0 and 1.0;
     */
    public void periodic() {
        motor.set(speed);
    }

    /**
     * <p> Sets the speed of the motor as a value between -1.0 and 1.0.
     * <p> This will be used to tighten and loosen the force on the string inside the climber.
     * @param speed The speed to set the motor at as a value between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
