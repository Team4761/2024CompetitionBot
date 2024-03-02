package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;


public class SwerveModuleTalon extends SubsystemBase{
    private TalonFX drive;
    private CANSparkMax steer;
    private CANcoder encoder;
    private double offset;

    private double sM;
    private final double driveConversionFactor = 0.0525772192354474; //0.0388385473906813; // This converts the encoders arbitrary units to meters travelled by the motor. The seemingly magic number below was gotten by driving the robot to 6m, looking at the odometry, and dividing 6 by the measured distance.
    private final double TOLERANCE_VOLTAGE_STEER = 0.25; // The minimum speed the steer should be able to get to in voltage. This is to prevent jittering.

    private double kS = 0.15; //kS feedforward for rotate motor

    // m/s, rotation2d
    private SwerveModuleState targetState = new SwerveModuleState();

    // pass in a ff+pid object or something, o is offset in radians
    /**
     * Creates a new SwerveModuleNeo object connected to a CanSparkMax encoder
     * 
     * @param driveID CAN port for motor driving wheel 
     * @param steerID CAN port for motor rotating wheel
     * @param encoderID DIO (?) port for CANSparkMax encoder
     * @param o Motor offset in degrees
     * @param invertDrive inverts drive motor if true
     * @param steerMult Steer (rotation) multiplier
     */
    public SwerveModuleTalon(int driveID, int steerID, int encoderID, double o, boolean invertDrive, double steerMult) {
        drive = new TalonFX(driveID);
        steer = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = new CANcoder(encoderID);

        // The seemingly magic number below was gotten by driving the robot to 6m, looking at the odometry, and dividing 6 by the measured distance.

        offset = o;

        drive.setInverted(invertDrive);

        // multiplier prob just for reversing
        sM = steerMult;

        //applySmartMotion();
    }
    
    public SwerveModuleTalon(int driveID, int steerID, int encoderID, double o, boolean invertDrive, double steerMult, double kS) {
        this(driveID, steerID, encoderID, o, invertDrive, steerMult);
        this.kS = kS;
    }

    public SwerveModuleTalon(int driveID, int steerID, int encoderID, double o, boolean invertDrive, double steerMult, boolean simplePID) {
        this(driveID,steerID,encoderID,o,invertDrive,steerMult);
        if(simplePID) setPIDValues(0.1, 0.0, 0.1, 0.1, 0.0, 0.1);
    }

    /**
     * 
     * Creates a new SwerveModuleNeo object connected to a CanSparkMax encoder
     * 
     * @param driveID CAN port for motor driving wheel 
     * @param steerID CAN port for motor rotating wheel
     * @param encoderID DIO (?) port for CANSparkMax encoder
     * @param o Motor offset in Radians
     * @param invertDrive inverts drive motor if true
     * @param steerMult Steer (rotation) multiplier
     * @param pSteer Proportional value for steer motor
     * @param iSteer Integration value for steer motor
     * @param dSteer Derivative value for steer motor
     * @param pDrive Proportional value for drive motor
     * @param iDrive Proportional value for drive motor
     * @param dDrive Proportional value for drive motor
     */
    public SwerveModuleTalon(int driveID, int steerID, int encoderID, double o, boolean invertDrive, double steerMult, double pSteer, double iSteer, double dSteer, double pDrive, double iDrive, double dDrive) {
        this(driveID,steerID,encoderID,o,invertDrive,steerMult);
        setPIDValues(pSteer, iSteer, dSteer, pDrive, iDrive, dDrive);
    }

    /**
     * Applies PID values to the sparkmaxes
     * See {@link SwerveModuleNeo} for where this function is used
     * 
     * @param pSteer Proportional value for steer motor
     * @param iSteer Integration value for steer motor
     * @param dSteer Derivative value for steer motor
     * @param pDrive Proportional value for drive motor
     * @param iDrive Proportional value for drive motor
     * @param dDrive Proportional value for drive motor
     */
    public void setPIDValues(double pSteer, double iSteer, double dSteer, double pDrive, double iDrive, double dDrive) {
        // Error! I'll just throw this out now so the team knows the issue ahead of time
        if(drive == null || steer == null) throw new NullPointerException("Attempting to write PID values to drive and steer motor before SwerveModuleNeo is initialized! Go to SwerveModuleNeo.java for more! Allister you suck!!!");
        SparkPIDController steerPIDController = steer.getPIDController();

        steerPIDController.setP(pSteer);
        steerPIDController.setI(iSteer);
        steerPIDController.setD(dSteer);
    }

    /**
     * Applies FeedForward to motors to counter mechanical issues (friction, countering gravity, controller input to small, etc)
     * @param fSteer FeedForward for steer motor
     * @param fDrive FeedForward for drive motor
     */
    public void setFeedForward(double fSteer, double fDrive) {
        // Error! I'll just throw this out now so the team knows the issue ahead of time
        if(drive == null || steer == null) throw new NullPointerException("Attempting to write FeedForward values to drive and steer motor before SwerveModuleNeo is initialized! Go to SwerveModuleNeo.java for more! Allister you suck!!!");
        SparkPIDController steerPIDController = steer.getPIDController();


        // drive.set(fDrive);
        steerPIDController.setFF(fSteer);
    }

    /**
     * <p>Applies the CANSparkMaxes built in SmartMotion system
     * <p>What this means is that rather than accelerating instantly, we smooth out the acceleration so the motors try not to rev up to max speed instantly
     * <p>This may not be entirely nessecary to apply so only use it if you feel like the robot is too shaky
     */
    public void applySmartMotion() {
        // Error! I'll just throw this out now so the team knows the issue ahead of time
        if(drive == null || steer == null) throw new NullPointerException("Attempting to write PID values to drive and steer motor before SwerveModuleNeo is initialized! Go to SwerveModuleNeo.java for more! Allister you suck!!!");
        SparkPIDController steerPIDController = steer.getPIDController();

        //drivePIDController.setSmartMotionAllowedClosedLoopError(0.1,0);

        steerPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,0);
        //steerPIDController.setSmartMotionAllowedClosedLoopError(0.1,0);
    }

    public TalonFX getDriveMotor() {
        return drive;
    }
    public CANSparkMax getSteerMotor() {
        return steer;
    }

    /**
     * Responsible for driving one SwerveModuleNeo
     * <p>This function reads from the targetState object for an angle (Unit unsure) and a velocity (Meters per Second)
     * <p>Then those numbers are send to the CANSparkMaxes so it can reach the desired angle and speed
     * 
     * <p>Keep in mind that this function has a lot of unexplained "magic numbers" 
     * <p>For more information track down Ian and ask him what the hell he wrote
     */
    public void go() {
        //System.out.println("speed: "+targetState.speedMetersPerSecond);
        // get to the set positions 
            
        //System.out.println(targetState.angle.getDegrees()+", "+getRotation().getDegrees()+", "+sM);

        //both need a P value to adjust it to the right speed
        double steerA = MathStuff.subtract(targetState.angle, getRotation()).getRotations()*sM*60;
        double steerB = Math.signum(steerA)*kS;
        SmartDashboard.putNumber("FL Voltage", steerA+steerB);
        // If the speed is less than the tolerance, it shouldn't rotate at all. This is to prevent jittering
        if (Math.abs(steerA + steerB) <= Robot.getShuffleboard().getSettingNum("Swerve Steer Tolerance")) {
            steer.setVoltage(0);
        }
        else {
            steer.setVoltage(steerA+steerB);
        }

        //if(true)
        double driveA = targetState.speedMetersPerSecond*10;
        double driveB = Math.signum(driveA)*0.36;
        drive.setVoltage(driveA+driveB);
    }

    public void setSpeeds(double d, double s) {
        drive.set(d);
        steer.set(s*sM);
    }

    public SwerveModulePosition getPosition() {
        SmartDashboard.putNumber("Module Position", drive.getPosition().getValueAsDouble());
        return new SwerveModulePosition(
            drive.getPosition().getValueAsDouble()*driveConversionFactor,   // The meters that the wheel has moved
            MathStuff.negative(getRotation()) // 2048 ticks to radians is 2pi/2048
        );
    }
    public double getDriveVelocity() { //rpms default supposedy, actual drive speed affected by gear ratio and wheel circumfernce
        return drive.getVelocity().getValueAsDouble(); //*gearratio*circumference=m/s except needs units adjustment
    }
    public double getSteerVelocity() { //rpms default, affected by gear ratio
        return steer.getEncoder().getVelocity(); // /gearratio=rpms of the wheel spinning
    }
    public Rotation2d getRotation() {
        // multiplied by 360 because it says its in rotations, unsure if correct
        // switch ((int)offset) {
        //     case 0:
        //         return new Rotation2d((encoder.getAbsolutePosition().getValueAsDouble()*360 + SmartDashboard.getNumber("FL Offset", 0.0) + 90) * 0.0174533);
        //     case 1:
        //         return new Rotation2d((encoder.getAbsolutePosition().getValueAsDouble()*360 + SmartDashboard.getNumber("FR Offset", 0.0) + 90) * 0.0174533);
        //     case 2:
        //         return new Rotation2d((encoder.getAbsolutePosition().getValueAsDouble()*360 + SmartDashboard.getNumber("BL Offset", 0.0) + 90) * 0.0174533);
        //     case 3:
        //         return new Rotation2d((encoder.getAbsolutePosition().getValueAsDouble()*360 + SmartDashboard.getNumber("BR Offset", 0.0) + 90) * 0.0174533);
        //     default:
        //     break;
            
        // }

        // Previous magic number: 0.0174533
        // This takes the encoder native rotation units and converts it to degrees (multiplies by 360), applies an offet, and then converts it to radians
        return new Rotation2d(Units.degreesToRadians(encoder.getAbsolutePosition().getValueAsDouble()*360 + offset));   // Converts the encoder ticks into radians after applying an offset.
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(drive.getVelocity().getValueAsDouble()*Constants.KRAKEN_RPM_TO_MPS_CONVERSION, getRotation());
    }

    // -1 to 1
    /*public void setSpeeds(double driveSpeed, double turnSpeed) {

        // check controlmodes for maybe useful or cool stuff
        drive.set(TalonFXControlMode.PercentOutput, driveSpeed);
        drive.set(TalonFXControlMode.PercentOutput, turnSpeed);
    }*/
    
    // default use optimization
    public void setTargetState(double metersPerSecond, Rotation2d angle) {
        targetState = new SwerveModuleState(metersPerSecond, angle);
        SwerveModuleState.optimize(targetState, getRotation());
    }
    public void setTargetState(double metersPerSecond, Rotation2d angle, boolean optimize) {
        targetState = new SwerveModuleState(metersPerSecond, angle);
        if(optimize) SwerveModuleState.optimize(targetState, getRotation());
    }
    public void setTargetState(SwerveModuleState target, boolean optimize) {
        targetState = target;
        if(optimize) targetState = SwerveModuleState.optimize(targetState, getRotation());
    }
}