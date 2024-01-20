package frc.robot.subsystems.westcoast;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// PLEASE NOTE THIS IS ONLY FOR TESITNG
public class WestCoastSubsystem {
    Timer timer = new Timer();
    double lastTime;
    double outputL = 0;
    double outputR = 0;


    public static TalonFX front_left = new TalonFX(8);
    public static TalonFX front_right = new TalonFX(0);
    public static TalonFX back_left = new TalonFX(7);
    public static TalonFX back_right = new TalonFX(1);

    private final PIDController left_PIDController = new PIDController(.1, 0, .001);
    private final PIDController right_PIDController = new PIDController(.1, 0, .001);

    public final MotorControllerGroup m_leftMotors = new MotorControllerGroup(front_left, back_left);
    public final MotorControllerGroup m_rightMotors = new MotorControllerGroup(front_right, back_right);

    private final SimpleMotorFeedforward m_feedFoward = new SimpleMotorFeedforward(0.106, 0.76);

    public final PIDController linear_PIDcontroller = new PIDController(0.01, 0, 0.1);
    public final PIDController angular_PIDcontroller = new PIDController(0.01, 0, 0.1);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.5);


    public WestCoastSubsystem() {
        m_leftMotors.setInverted(true);
        lastTime = timer.get();
    }


    // public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    //     final double leftFeedforward = m_feedFoward.calculate(speeds.leftMetersPerSecond);
    //     final double rightFeedforward = m_feedFoward.calculate(speeds.rightMetersPerSecond);

    //     final double leftOutput =
    //             left_PIDController.calculate(nativeUnitsToDistanceMeters(averageMotorGroupVelocity(front_left, back_left)), 1);
    //     final double rightOutput =
    //             right_PIDController.calculate(nativeUnitsToDistanceMeters(averageMotorGroupVelocity(front_right, back_right)), 1);
    
    //     m_leftMotors.set(leftOutput);
    //     m_rightMotors.set(rightOutput);
    // }

    // public double averageMotorGroupVelocity(TalonFX front_motor, TalonFX back_motor) {
    //     //Needs to account for gear ratio
    //     return ((front_motor.getVelocity().getValueAsDouble() + back_motor.getVelocity().getValueAsDouble()) / 2);
    // }

    // private double nativeUnitsToDistanceMeters(double sensorCounts) {
    //     double motorRotations = (double) sensorCounts / 2048;
    //     double wheelRotations = motorRotations / 8.0;
    //     double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2.0));
    //     return positionMeters;
    // }

    // // NOT USED
    // public void drive(double xSpeed, double rot) {
    //     var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    //     setSpeeds(wheelSpeeds);
    // }


    public void setVoltages(double left, double right) {
        if (m_leftMotors.getInverted()) {
            left *= -1;
        }
        if (m_rightMotors.getInverted()){
            right *= -1;
        }
        front_left.setVoltage(left);
        front_right.setVoltage(right);
        back_left.setVoltage(left);
        back_right.setVoltage(right);
    }

    public void arcadeDrive(double speed, double rotation) {
        DifferentialDrive.WheelSpeeds wheelSpeeds = arcadeDriveIK((speed+Math.signum(speed)*0.25)/1.25, -(rotation)*0.9);
        
        // changing to make acceleration only take as long as the side that accelerates the least
        // so when going from straight to turning, one side would not be limited while the other isn't

        // the time it would take for the lower change side is t = lowerSpeedNeeded/MAXACCEL
        // the maxChange to get the higher change side to that is higherSpeedNeeded/t*(timer.get()-lastTime)

        double lowerMaxChange = Math.abs((timer.get()-lastTime) * 4 * 1); // lower max change for lower desired-current
        double higherMaxChange;

        if (wheelSpeeds.left*1.5-outputL != 0 && wheelSpeeds.right*1.5-outputR!=0) {
            // higherNeeded*maxAccel/lowerneeded*deltaT
            higherMaxChange = Math.max(Math.abs(wheelSpeeds.left*1.5-outputL),Math.abs(wheelSpeeds.right*1.5-outputR)) * 4 / Math.min(Math.abs(wheelSpeeds.left*1.5-outputL),Math.abs(wheelSpeeds.right*1.5-outputR)) * (timer.get()-lastTime);
        } else {
            // very high
            higherMaxChange = 10000;
        }
        
        // if left needs less change
        if (Math.abs(outputL-wheelSpeeds.left*1.5)<Math.abs(outputR-wheelSpeeds.right*1.5)) {
            outputL = MathUtil.clamp(wheelSpeeds.left*1.5, outputL-lowerMaxChange, outputL+lowerMaxChange);
            outputR = MathUtil.clamp(wheelSpeeds.right*1.5, outputR-higherMaxChange, outputR+higherMaxChange);
        } else {
            outputL = MathUtil.clamp(wheelSpeeds.left*1.5, outputL-higherMaxChange, outputL+higherMaxChange);
            outputR = MathUtil.clamp(wheelSpeeds.right*1.5, outputR-lowerMaxChange, outputR+lowerMaxChange);
        }

        double vL = outputL*5 + Math.signum(outputL)*0.3 + 0.1 * (outputL-getLeftVelocity()*0.00001948155);
        double vR = outputR*5 + Math.signum(outputR)*0.2 + 0.1 * (outputR-getRightVelocity()*0.00001948155);

        lastTime = timer.get();

        setVoltages(vL, vR);
    }

    DifferentialDrive.WheelSpeeds arcadeDriveIK(double xSpeed, double zRotation) {
        // removed deadzone here because applied in xboxcontrol
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        // Square the inputs (while preserving the sign) to increase fine control
        zRotation = Math.copySign(Math.pow(zRotation, 3), zRotation);
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);

        double leftSpeed = xSpeed - zRotation;
        double rightSpeed = xSpeed + zRotation;

        // Find the maximum possible value of (throttle + turn) along the vector
        // that the joystick is pointing, then desaturate the wheel speeds
        double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
        double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
        if (greaterInput == 0.0) {
            return new DifferentialDrive.WheelSpeeds(0.0, 0.0);
        }
        double saturatedInput = (greaterInput + lesserInput/*  /1.2  */) / greaterInput; //see if should divide lesserInput by ~1.2 to slightly increase speed of corner joystic inputs
        leftSpeed /= saturatedInput;
        rightSpeed /= saturatedInput;

        return new DifferentialDrive.WheelSpeeds(leftSpeed, rightSpeed);
    }



    public static double getLeftVelocity() {
        //Needs to account for gear ratio
        return -((front_left.getVelocity().getValueAsDouble() + back_left.getVelocity().getValueAsDouble()) / 2);
    }
    public static double getRightVelocity() {
        //Needs to account for gear ratio
        return ((front_right.getVelocity().getValueAsDouble() + back_right.getVelocity().getValueAsDouble()) / 2);
    }
}
