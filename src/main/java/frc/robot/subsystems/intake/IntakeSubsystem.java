package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{ 
    // Neos that actually intake (left or right facing forward)
    private CANSparkMax intakeT;
    private CANSparkMax intakeB;

    private CANSparkMax angleMotorLeft; // Motor for angling the shooter up and down, assuming that the front of the shooter is the forward direction

    private DutyCycleEncoder encoder;

    private PIDController anglePID;         // Will be used to get the shooter a desired angle.
    private ArmFeedforward angleFeedForward;// Will be used to maintain the shooter's angle.

    private Rotation2d targetAngle = new Rotation2d(); // The angle the intake should get to where 0 degrees is (undecided).

    private static double INTAKE_ANGLE_OFFSET = Units.degreesToRadians(-180);    // Should be set such that when the arm is fully outstretched (perpendicular with the ground), the encoder measures 0 radians/degrees. This is in arbitrary encoder units.


    public IntakeSubsystem() {
        intakeB = new CANSparkMax(Constants.INTAKE_BOT_PORT, MotorType.kBrushless);
        intakeT = new CANSparkMax(Constants.INTAKE_TOP_PORT, MotorType.kBrushless);
        angleMotorLeft = new CANSparkMax(Constants.INTAKE_ANGLE_LEFT_MOTOR_PORT, MotorType.kBrushless);

        targetAngle = new Rotation2d(Constants.INTAKE_START_POSITION);

        encoder = new DutyCycleEncoder(3);

        anglePID = new PIDController(.30, 0, 0);  // These values have yet to be tuned.
        angleFeedForward = new ArmFeedforward(0,0, 0); //ks = 0, kg = 0.91, kv = 1.95// Placeholder values. Can be tuned or can use https://www.reca.lc/ to tune.

    }

    public void periodic() {
        // getIntakeToSetAngle();

        SmartDashboard.putNumber("Intake Setpoint Desired", targetAngle.getDegrees());
        SmartDashboard.putNumber("Intake Angle", getIntakeAngle().getDegrees());
    }


    /**
     * <p> Gets the intake to {targetAngle} radians using PID and Feed Forward.
     * <p> This must be called during the periodic function to work.
     */
    public void getIntakeToSetAngle() {
        double currentAngle = getIntakeAngle().getRadians();
        double currentVelocity = getIntakeAngleVelocity();
        double speed = anglePID.calculate(currentAngle, targetAngle.getRadians()) + angleFeedForward.calculate(targetAngle.getRadians(), 0.0);

        // Neither of the below have been tested (i.e. idk which one should be reversed rn)
        angleMotorLeft.set(speed);
    }

    /**
     * <p>This runs the intake motors so that it actually intakes.
     * @param speed The speed to run the motors as a number between 0.0 to 1.0
     */
    public void intake(double speed) {
        intakeB.set(-speed);
        intakeT.set(-speed);
    }

    /**
     * <p> This runs the outtake motors so that it spits out whatever it has in it.
     * @param speed The speed to run the motors at as a number between 0.0 to 1.0
     */
    public void outtake(double speed) {
        intakeB.set(speed);
        intakeT.set(speed);
    }

    /**
     * <p> This sets the target rotation to what it currently is plus the offset in radians where up is positive and down is negative.
     * @param offsetRadians
     */
    public void rotate(double offsetRadians) {
        targetAngle = new Rotation2d(targetAngle.getRadians() + offsetRadians);
    }

    public void setAngleMotorSpeed(double speed){
        angleMotorLeft.set(speed);
    }

    /**
     * <p> This sets the target rotation of the intake to {rotation} and will get to that rotation during its periodic function where up is positive and down is negative.
     * @param rotation The new rotation to get to.
     */
    public void goToRotation(Rotation2d rotation) {
        targetAngle = rotation;
    }

    /**
     * <p> Determines the angle of the shooter based off of the left motor's current position after applying an offset.
     * @return the angle of the shooter in radians where up is positive and 0 radians is perpendicular with the ground.
     */
    public Rotation2d getIntakeAngle() {
        return new Rotation2d(encoder.getAbsolutePosition() * Constants.ENCODER_UNITS_TO_RADIANS + INTAKE_ANGLE_OFFSET);
    }

    /**
     * <p> Gets the current moving velocity of the angle mechanism of the shooter.
     * @return The speed at which the shooter's angle changes in meters per second.
     */
    public double getIntakeAngleVelocity() {
        return (angleMotorLeft.getEncoder().getVelocity() * Constants.SHOOTER_RPM_TO_MPS);
    }

    public void stop(){
        intakeB.set(0);
        intakeT.set(0);
    }
}