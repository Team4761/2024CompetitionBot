package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkLowLevel;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class SwerveModuleNeo extends SubsystemBase{
    private CANSparkMax drive;
    private CANSparkMax steer;
    private CANCoder encoder;
    private double offset;

    private double dM;
    private double sM;

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
     * @param driveMult Drive multiplier
     * @param steerMult Steer (rotation) multiplier
     */
    public SwerveModuleNeo(int driveID, int steerID, int encoderID, double o, double driveMult, double steerMult) {
        drive = new CANSparkMax(driveID, CANSparkLowLevel.MotorType.kBrushless);
        steer = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = new CANCoder(encoderID);

        offset = o;


        // multiplier prob just for reversing
        dM = driveMult;
        sM = steerMult;

        //applySmartMotion();
    }

    public SwerveModuleNeo(int driveID, int steerID, int encoderID, double o, double driveMult, double steerMult, boolean simplePID) {
        this(driveID,steerID,encoderID,o,driveMult,steerMult);
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
     * @param driveMult Drive multiplier
     * @param steerMult Steer (rotation) multiplier
     * @param pSteer Proportional value for steer motor
     * @param iSteer Integration value for steer motor
     * @param dSteer Derivative value for steer motor
     * @param pDrive Proportional value for drive motor
     * @param iDrive Proportional value for drive motor
     * @param dDrive Proportional value for drive motor
     */
    public SwerveModuleNeo(int driveID, int steerID, int encoderID, double o, double driveMult, double steerMult, double pSteer, double iSteer, double dSteer, double pDrive, double iDrive, double dDrive) {
        this(driveID,steerID,encoderID,o,driveMult,steerMult);
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
        SparkPIDController drivePIDController = drive.getPIDController();
        SparkPIDController steerPIDController = steer.getPIDController();

        drivePIDController.setP(pDrive);
        drivePIDController.setI(iDrive);
        drivePIDController.setD(dDrive);

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
        SparkPIDController drivePIDController = drive.getPIDController();
        SparkPIDController steerPIDController = steer.getPIDController();

        drivePIDController.setFF(fDrive);
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
        SparkPIDController drivePIDController = drive.getPIDController();
        SparkPIDController steerPIDController = steer.getPIDController();

        drivePIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,0);
        //drivePIDController.setSmartMotionAllowedClosedLoopError(0.1,0);

        steerPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,0);
        //steerPIDController.setSmartMotionAllowedClosedLoopError(0.1,0);
    }

    public CANSparkMax getDriveMotor() {
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
        double steerA = MathStuff.subtract(targetState.angle, getRotation()).getRotations()*sM*2.5;
        double steerB = Math.signum(steerA)*0.008;
        steer.set(steerA+steerB);

        //if(true)
        double driveA = targetState.speedMetersPerSecond*dM*0.8;
        double driveB = Math.signum(driveA)*0.03;
        drive.set(driveA+driveB);
    }

    public void setSpeeds(double d, double s) {
        drive.set(d*dM);
        steer.set(s*sM);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drive.getEncoder().getPosition(), 
            getRotation() // 2048 ticks to radians is 2pi/2048
        );
    }
    public double getDriveVelocity() { //rpms default supposedy, actual drive speed affected by gear ratio and wheel circumfernce
        return drive.getEncoder().getVelocity(); //*gearratio*circumference=m/s except needs units adjustment
    }
    public double getSteerVelocity() { //rpms default, affected by gear ratio
        return steer.getEncoder().getVelocity(); // /gearratio=rpms of the wheel spinning
    }
    public Rotation2d getRotation() {
        return new Rotation2d((encoder.getAbsolutePosition() + offset + 90) * 0.0174533);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(drive.getEncoder().getVelocity()*Constants.RPM_TO_MPS_CONVERSION,getRotation());
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