package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX shooterLeft;        // Motor for the left of the actual shooter, assuming that the front of the shooter is the forward direction
    private TalonFX shooterRight;       // Motor for the right of the actual shooter, assuming that the front of the shooter is the forward direction
    private CANSparkMax intakeLeft;     // Motor for the left intake of the shooter, assuming that the front of the shooter is the forward direction
    private CANSparkMax intakeRight;    // Motor for the right intake of the shooter, assuming that the front of the shooter is the forward direction
    private CANSparkMax angleMotorLeft; // Motor for angling the shooter up and down, assuming that the front of the shooter is the forward direction
    private CANSparkMax angleMotorRight;// Motor for angling the shooter up and down, assuming that the front of the shooter is the forward direction

    private PIDController anglePID;         // Will be used to get the shooter a desired angle.
    private ArmFeedforward angleFeedForward;// Will be used to maintain the shooter's angle.

    // The break beam sensors are from https://www.adafruit.com/product/2168
    private DigitalInput intakeUpperSensor;    // This is the break beam sensor right before the top of the shooter
    private DigitalInput intakeLowerSensor;    // This is the break beam sensor in between the shooter and the actual intake

    private double targetSpeed;         // Shooting speed in rotations of the wheel / second
    private double targetAngle;         // Shooting angle in radians. The origin should be when the shooter is perpendicular with the ground (flat and fully outstretched).

    private final double SHOOTER_ANGLE_OFFSET = 0.0;  // Should be set such that when the arm is fully outstretched (perpendicular with the ground), the encoder measures 0 radians/degrees. This is in arbitrary encoder units.



    public ShooterSubsystem() {
        shooterLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR_PORT);
        shooterRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR_PORT);
        intakeLeft = new CANSparkMax(Constants.SHOOTER_INTAKE_LEFT_MOTOR_PORT, MotorType.kBrushless);
        intakeRight = new CANSparkMax(Constants.SHOOTER_INTAKE_RIGHT_MOTOR_PORT, MotorType.kBrushless);
        angleMotorLeft = new CANSparkMax(Constants.SHOOTER_ANGLE_LEFT_MOTOR_PORT, MotorType.kBrushless);
        angleMotorRight = new CANSparkMax(Constants.SHOOTER_ANGLE_RIGHT_MOTOR_PORT, MotorType.kBrushless);

        anglePID = new PIDController(1.0, 0.0, 0.0);    // Placeholder values, has yet to be tuned.
        angleFeedForward = new ArmFeedforward(0.0, 0.91, 1.95);   // Placeholder values. Can be tuned or can use https://www.reca.lc/ to tune.

        intakeUpperSensor = new DigitalInput(Constants.SHOOTER_SENSOR_UPPER_PORT);
        intakeLowerSensor = new DigitalInput(Constants.SHOOTER_SENSOR_LOWER_PORT);

        targetSpeed = 0.0;
        targetAngle = 0.0;
    }


    public void periodic() {
        getShooterToSetSpeed();     // Gets the shooter to speed up to {targetSpeed} rotations per second.
        getShooterToSetAngle();     // Gets the shooter to angle at {targetAngle} radians.
    }



    /**
     * <p> Uses some basic PID and feedforwards to get the shooter to spin at a set speed in rotations per second.
     * <p> This must be called during the periodic function to work.
     */
    public void getShooterToSetSpeed() {
        double avgEncoderSpd = (shooterLeft.getVelocity().getValueAsDouble()+shooterRight.getVelocity().getValueAsDouble())/2; //rotations per second allegedly

        double accelFactor = 0.005 * (targetSpeed-avgEncoderSpd); // P kinda
        double feedForwardV = 0.1 * targetSpeed; //magic numbers no math or testing done yet, ideally sets the motors to near the velocity
        double feedForwardS = 0; //whatever number to maintain the speed
        
        double spdOut = accelFactor + feedForwardV + feedForwardS;

        shooterLeft.set(spdOut); //probably test then use setVoltage
        shooterRight.set(spdOut);  // As of Jan 20, 2024, the speeds are not reversed
    }

    /**
     * <p> Gets the shooter to {targetAngle} radians using PID and Feed Forward.
     * <p> This must be called during the periodic function to work.
     */
    public void getShooterToSetAngle() {
        double currentAngle = getShooterAngle();
        double currentVelocity = getShooterAngleVelocity();
        double speed = anglePID.calculate(currentAngle, targetAngle) + angleFeedForward.calculate(targetAngle, 0.0);

        // Neither of the below have been tested (i.e. idk which one should be reversed rn)
        angleMotorLeft.set(speed);
        angleMotorRight.set(-speed);
    }

    /**
     * <p> Sets the SHOOTING speed in rotations per second (hopefully), does not change until stated otherwise
     * @param speed The speed of the shooter in rotations per second.
     */
    public void setShooterSpeed(double speed) {
        targetSpeed = speed;
    }

    /**
     * <p> Sets the target angle for the shooter to be oriented in (looking up or down)
     * @param angleRadians The new angle to get to in radians where 0 radians is fully outstretched and positive radians is upwards.
     */
    public void setShooterAngle(double angleRadians) {
        targetAngle = angleRadians;
    }

    /**
     * <p> Sets the target angle for the shooter to what it currently is plus {angleRadians} where up is positive and down is negative
     * @param angleRadians The offset the shooter angle should get to in radians.
     */
    public void rotate(double angleRadians) {
        targetAngle += angleRadians;
    }

    /**
     * <p> Sets the speed of the intake to the motor's speed. (the belt that pulls the game piece from the actual intake)
     * @param speed The speed of the intake as a number between -1.0 and 1.0 inclusive which represents 100% speed outtake and intake respectively.
     */
    public void setIntakeSpeed(double speed) {
        intakeLeft.set(speed);
        intakeRight.set(-speed);
    }

    /**
     * <p> Determines the angle of the shooter based off of the left motor's current position after applying an offset.
     * @return the angle of the shooter in radians where up is positive and 0 radians is perpendicular with the ground.
     */
    public double getShooterAngle() {
        return (angleMotorLeft.getEncoder().getPosition() - SHOOTER_ANGLE_OFFSET) * Constants.NEO_UNITS_TO_RADIANS;
    }

    /**
     * <p> Gets the current moving velocity of the angle mechanism of the shooter.
     * @return The speed at which the shooter's angle changes in meters per second.
     */
    public double getShooterAngleVelocity() {
        return (angleMotorLeft.getEncoder().getVelocity() * Constants.SHOOTER_RPM_TO_MPS);
    }

    /**
     * <p> Using the break beam sensor, this returns whether or not there is a piece in the upper shooter.
     * @return True if there is a piece in the upper shooter. False if there is none.
     */
    public boolean isPieceInUpperIntake() {
        return intakeUpperSensor.get();
    }

    /**
     * <p> Using the break beam sensor, this returns whether or not there is a piece in the lower shooter.
     * @return True if there is a piece in the lower shooter. False if there is none.
     */
    public boolean isPieceInLowerIntake() {
        return intakeLowerSensor.get();
    }
}
