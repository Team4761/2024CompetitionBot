package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.MathStuff;


public class ShooterSubsystem extends SubsystemBase {
    private TalonFX shooterLeft;        // Motor for the left of the actual shooter, assuming that the front of the shooter is the forward direction
    private TalonFX shooterRight;       // Motor for the right of the actual shooter, assuming that the front of the shooter is the forward direction
    private CANSparkMax intakeLeft;     // Motor for the left intake of the shooter, assuming that the front of the shooter is the forward direction
    private CANSparkMax intakeRight;    // Motor for the right intake of the shooter, assuming that the front of the shooter is the forward direction
    private CANSparkMax angleMotorRight;// Motor for angling the shooter up and down, assuming that the front of the shooter is the forward direction

    private DutyCycleEncoder encoder;    // Absolute encoder for the angle of the shooter

    private PIDController anglePID;         // Will be used to get the shooter a desired angle.
    private ArmFeedforward angleFeedForward;// Will be used to maintain the shooter's angle.

    // The break beam sensors are from https://www.adafruit.com/product/2168
    private DigitalInput intakeUpperSensor;    // This is the break beam sensor right before the top of the shooter
    private DigitalInput intakeLowerSensor;    // This is the break beam sensor in between the shooter and the actual intake

    private double targetSpeed;         // Shooting speed in rotations of the wheel / second
    private double targetAngle;         // Shooting angle in radians. The origin should be when the shooter is perpendicular with the ground (flat and fully outstretched).

    private final double SHOOTER_ANGLE_OFFSET = 0.6351;  // Should be set such that when the arm is fully outstretched (perpendicular with the ground), the encoder measures 0 radians/degrees. This is in arbitrary encoder units.


    public ShooterSubsystem() {
        shooterLeft = new TalonFX(Constants.SHOOTER_LEFT_MOTOR_PORT); //top and bottom no?
        shooterRight = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR_PORT);
        intakeLeft = new CANSparkMax(Constants.SHOOTER_INTAKE_LEFT_MOTOR_PORT, MotorType.kBrushless);
        intakeRight = new CANSparkMax(Constants.SHOOTER_INTAKE_RIGHT_MOTOR_PORT, MotorType.kBrushless);
        angleMotorRight = new CANSparkMax(Constants.SHOOTER_ANGLE_RIGHT_MOTOR_PORT, MotorType.kBrushless);

        encoder = new DutyCycleEncoder(2); //needs port
        //right now 0 is parallel to ground and increases going up

        anglePID = new PIDController(2.8, 0.01, 0.0);    // Placeholder values, has yet to be tuned.
        angleFeedForward = new ArmFeedforward(0,0.2,1.1,0.01); //recalc numbers with questionable inputs
        
        //ks = 0, kg = 0.91, kv = 1.95    // Placeholder values. Can be tuned or can use https://www.reca.lc/ to tune.

        intakeUpperSensor = new DigitalInput(Constants.SHOOTER_SENSOR_UPPER_PORT);
        intakeLowerSensor = new DigitalInput(Constants.SHOOTER_SENSOR_LOWER_PORT);

        targetSpeed = 0.0;
        targetAngle = Units.degreesToRadians(63); //63 gets to 55ish

        intakeRight.setInverted(true);
    }


    public void periodic() {
        getShooterToSetSpeed();     // Gets the shooter to speed up to {targetSpeed} rotations per second.
        getShooterToSetAngle();     // Gets the shooter to angle at {targetAngle} radians.
        
        SmartDashboard.putNumber("Shooter Speed L", shooterLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Speed R", shooterRight.getVelocity().getValueAsDouble());
        
        SmartDashboard.putNumber("Shooter Angle", getShooterAngle().getDegrees());

        SmartDashboard.putNumber("Shooter Setpoint Desired", Units.radiansToDegrees(targetAngle));

        SmartDashboard.putBoolean("Breakbeam lower", isPieceInLowerIntake());
        SmartDashboard.putBoolean("Breakbeam upper", isPieceInUpperIntake());
    }



    /**
     * <p> Uses some basic PID and feedforwards to get the shooter to spin at a set speed in rotations per second.
     * <p> This must be called during the periodic function to work.
     */
    
    private SimpleMotorFeedforward shootingFFTop= new SimpleMotorFeedforward(0.01, 0.112); //right
    private SimpleMotorFeedforward shootingFFBot= new SimpleMotorFeedforward(0.01, 0.115); //left

    public void getShooterToSetSpeed() {

        double kP = 0.08;

        double accelFactor = kP * (targetSpeed-shooterLeft.getVelocity().getValueAsDouble()); // P kinda
        accelFactor = MathUtil.clamp(-2, accelFactor, 2);
        double spdOut = accelFactor + shootingFFBot.calculate(targetSpeed);

        shooterLeft.setVoltage(spdOut); //probably test then use setVoltage
        
        //other half
        accelFactor = kP * (targetSpeed-shooterRight.getVelocity().getValueAsDouble()); // P kinda
        accelFactor = MathUtil.clamp(-2, accelFactor, 2);
        spdOut = accelFactor + shootingFFTop.calculate(targetSpeed);

        shooterRight.setVoltage(spdOut);  // As of Jan 20, 2024, the speeds are not reversed
    }


    public void setShooterAngleSpeed(double speed) {
        angleMotorRight.set(speed);
    }


    /**
     * <p> Gets the shooter to {targetAngle} radians using PID and Feed Forward.
     * <p> This must be called during the periodic function to work.
     */
    public void getShooterToSetAngle() {
        double currentAngle = getShooterAngle().getRadians();
        double currentVelocity = getShooterAngleVelocity();
        double speed = anglePID.calculate(currentAngle, targetAngle) + angleFeedForward.calculate(targetAngle, 0.9*(targetAngle-currentAngle));

        // Neither of the below have been tested (i.e. idk which one should be reversed rn)
        angleMotorRight.setVoltage(speed); //voltage because battery drain stuff
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
        // limit target angle so shooter doesnt go backwards
        targetAngle = MathUtil.clamp(0, targetAngle+angleRadians, Math.PI/2);
        
    }

    /**
     * <p> Sets the speed of the intake to the motor's speed. (the belt that pulls the game piece from the actual intake)
     * @param speed The speed of the intake as a number between -1.0 and 1.0 inclusive which represents 100% speed outtake and intake respectively.
     */
    public void setIntakeSpeed(double speed) {
        intakeLeft.set(speed);
        intakeRight.set(speed); // setted inverted to true
    }

    /**
     * <p> Determines the angle of the shooter based off of the left motor's current position after applying an offset.
     * @return the angle of the shooter in radians where up is positive and 0 radians is perpendicular with the ground.
     */
    public Rotation2d getShooterAngle() {
        return new Rotation2d((encoder.getAbsolutePosition() - SHOOTER_ANGLE_OFFSET)*Math.PI*2);
    }

    /**
     * <p> Gets the current moving velocity of the angle mechanism of the shooter.
     * @return The speed at which the shooter's angle changes in meters per second.
     */
    public double getShooterAngleVelocity() {
        return (angleMotorRight.getEncoder().getVelocity() * Constants.SHOOTER_RPM_TO_MPS);
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
