package frc.robot.subsystems.swerve;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Auto.PrintText;


/**
 * <p> This is the subsystem for Swerve which contains all of its motors, encoders, PID values, and more.
 * <p> All measurements are in meters and radians (but rotation generally uses Rotation2d which includes both radians and degrees)
 * <p> To make this consistent with PathPlanner and all other commands/odometry, the +x direction should be forwards and the +y direction should be left.
 */
public class SwerveDriveSubsystem extends SubsystemBase {

    SwerveModuleState[] targetStates = new SwerveModuleState[4];

    // motors offset in degrees && i think negative is ccw
    // private SwerveModuleTalon m_frontLeftModule  = new SwerveModuleTalon(Constants.FL_DRIVE_PORT , Constants.FL_ROTATE_PORT , Constants.FL_ENCODER_PORT , 83.30076,false,  -1.0);    // Formerly -54.5, 1.0,  1.0
    // private SwerveModuleTalon m_frontRightModule = new SwerveModuleTalon(Constants.FR_DRIVE_PORT , Constants.FR_ROTATE_PORT , Constants.FR_ENCODER_PORT ,   -61.74209, false, -1.0);      // Formerly -6, -1.0, -1.0
    // private SwerveModuleTalon m_backLeftModule   = new SwerveModuleTalon(Constants.BL_DRIVE_PORT , Constants.BL_ROTATE_PORT , Constants.BL_ENCODER_PORT ,  -69.78506, true, -1.0);       // Formerly -68, 1.0, -1.0
    // private SwerveModuleTalon m_backRightModule  = new SwerveModuleTalon(Constants.BR_DRIVE_PORT , Constants.BR_ROTATE_PORT , Constants.BR_ENCODER_PORT , -16.87491, true,  -1.0);      // Formerly 82, 1.0,  -1.0
    
    private SwerveModuleTalon m_frontLeftModule  = new SwerveModuleTalon(Constants.FL_DRIVE_PORT , Constants.FL_ROTATE_PORT , Constants.FL_ENCODER_PORT , -29.5313 ,false,  -1.0);    // Formerly -54.5, 1.0,  1.0
    private SwerveModuleTalon m_frontRightModule = new SwerveModuleTalon(Constants.FR_DRIVE_PORT , Constants.FR_ROTATE_PORT , Constants.FR_ENCODER_PORT ,   22.6757, false, -1.0);      // Formerly -6, -1.0, -1.0
    private SwerveModuleTalon m_backLeftModule   = new SwerveModuleTalon(Constants.BL_DRIVE_PORT , Constants.BL_ROTATE_PORT , Constants.BL_ENCODER_PORT ,  22.3242, true, -1.0);       // Formerly -68, 1.0, -1.0
    private SwerveModuleTalon m_backRightModule  = new SwerveModuleTalon(Constants.BR_DRIVE_PORT , Constants.BR_ROTATE_PORT , Constants.BR_ENCODER_PORT , 63.2812, true,  -1.0);      // Formerly 82, 1.0,  -1.0

    public static boolean isRobotRelative = false;

    private SwerveModulePosition[] m_swervePositions= new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(), 
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(), 
        m_backRightModule.getPosition()
    };

    // Gian: Has potential use cases involving pathplanner
    private SwerveModuleState[] m_swerveStates = new SwerveModuleState[] {
        m_frontLeftModule.getState(), 
        m_frontRightModule.getState(),
        m_backLeftModule.getState(), 
        m_backRightModule.getState()
    };

    private ADIS16470_IMU m_gyro = new ADIS16470_IMU();
    private Rotation2d gyroOffset = new Rotation2d();   // This is the offset of the gyro where 0 degrees is when the robot is facing away from the alliance wall (forwards).

    private SwerveDriveKinematics m_kinematics;         // Essentially the built-in math required to use Swerve

    private SwerveDriveOdometry m_odometry;             // Stores what the robot THINKS it's position is based on Swerve encoders.
    private Pose2d m_pose;                              // Stores the robots position ON THE FIELD where 0,0 is the STARTING POSITION of the robot when this subsystem is initialized

    private Rotation2d pointDir = new Rotation2d();     // The direction the robot thinks it measured to be looking at between 1-10 cycles ago.
    
    // field oriented driving states
    private double speedRot = 0;
    private double speedX = 0;
    private double speedY = 0;


    // positions of the wheels relative to center in meters.
    public SwerveDriveSubsystem (Translation2d fL, Translation2d fR, Translation2d bL, Translation2d bR) {
        m_kinematics = new SwerveDriveKinematics(fL, fR, bL, bR);   // Load the relative positions of all our swerve modules (wheels) in relation to the origin.
        m_pose = new Pose2d();  // Starts the position at 0,0
        m_odometry =  new SwerveDriveOdometry(m_kinematics, getGyroRotation(), m_swervePositions, m_pose); // Start the odometry at 0,0
        targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroRotation()));
    
        configureAutoBuilder();
    }

    /**
     *  <p> Holonomic (Swerve) autobuilder from pathplanner
     *  <p> See https://pathplanner.dev/pplib-build-an-auto.html#configure-autobuilder for more
     */
    public void configureAutoBuilder() {
        // This just gets the PID values of one motor. All 4 should be equal though!!
        // SparkPIDController spcd = m_backLeftModule.getDriveMotor().getPIDController(); // d for drive
        // SparkPIDController spcs = m_backLeftModule.getSteerMotor().getPIDController(); // s for steer
        // System.out.println("P: " + spcd.getP() + " I: " + spcd.getI() + " D: " + spcd.getD());
        // https://github.com/mjansen4857/pathplanner/wiki/Java-Example:-Build-an-Auto
        HolonomicPathFollowerConfig hpfc = new HolonomicPathFollowerConfig(
            new PIDConstants(
                1.2,0,0
                // spcd.getP(),spcd.getI(),spcd.getD()
                ), // Translation PID Constants
            new PIDConstants(
                0.01,0,0
                // spcs.getP(),spcs.getI(),spcs.getD()
                ), // Rotateion PID Constants
            Constants.DRIVETRAIN_MAX_SPEED_MPS, // max mps
            0.31115*Math.sqrt(2.0), //Distance from robot center to wheel in meters. They're all equidistant so this is a good value
            new ReplanningConfig() //
        );

        // This is static, so we are not just creating an object that is never used
        // TODO: confirm the parameters on this function
        AutoBuilder.configureHolonomic(
            this::getPoseForPathPlanner,  // +x must be forwards, +y must be left.
            this::resetPoseForPathPlanner, 
            this::getRobotRelativeSpeeds, 
            this::swerveDriveR, 
            hpfc,
            this::isOnRedAlliance,
            this
        );

        // This registers the commands that are used in the autos as events
        NamedCommands.registerCommand("PrintText", new PrintText());
    }
    

    @Override
    public void periodic() {
        m_swervePositions = new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(), 
            m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(), 
            m_backRightModule.getPosition()
        };

        m_swerveStates = new SwerveModuleState[] {
            m_frontLeftModule.getState(), 
            m_frontRightModule.getState(),
            m_backLeftModule.getState(), 
            m_backRightModule.getState()
        };
        
        // update pose

        m_pose = m_odometry.update(
            getGyroRotation(),
            m_swervePositions
        );
        m_pose = new Pose2d(m_pose.getX(), m_pose.getY(), m_pose.getRotation());   // The y value needs to be negative to make left +y

        //forward is +x. left is +y

        SmartDashboard.putNumber("Odometry x", m_pose.getX());
        SmartDashboard.putNumber("Odometry y", m_pose.getY());

        
        SmartDashboard.putNumber("Front Left Rot", m_frontLeftModule.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Front Right Rot", m_frontRightModule.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Back Left Rot", m_backLeftModule.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Back Right Rot", m_backRightModule.getPosition().angle.getDegrees());

        SmartDashboard.putNumber("Front Left Drive", m_frontLeftModule.getPosition().distanceMeters);
        SmartDashboard.putNumber("Front Right Drive", m_frontRightModule.getPosition().distanceMeters);
        SmartDashboard.putNumber("Back Left Drive", m_backLeftModule.getPosition().distanceMeters);
        SmartDashboard.putNumber("Back Right Drive", m_backRightModule.getPosition().distanceMeters);
        
        SmartDashboard.putNumber("Front Left Target", targetStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Front Right Target", targetStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Back Left Target", targetStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Back Right Target", targetStates[3].angle.getDegrees());

        SmartDashboard.putNumber("Gyro Angle", getGyroRotation().getDegrees());
        SmartDashboard.putNumber("speedX", speedX);
        SmartDashboard.putNumber("speedY", speedY);
        SmartDashboard.putNumber("speedRot", speedRot);
        //SmartDashboard.putNumber("GyroRotation", getGyroRotation().getRadians()); see this but degrees

        SmartDashboard.putBoolean("Robot Relative", isRobotRelative);

        if(lastDone==0) {                   // After moving for 10 cycles, check the rotation of the robot.
            pointDir = getGyroRotation();   // Radians
        }
        // if not turning do lock on
        if (speedRot == 0) {
            double rotP = getGyroRotation().minus((pointDir)).getDegrees()    // proportional to keep robot turned properly (finds the distance between expected and actual rotation to apply some opposite rotational speed)
                * (0.001 + 0.03*(Math.abs(speedX) + Math.abs(speedY)));     // If going faster, correct more (because small corrections are lost)
            if (Math.abs(rotP)<0.001 || lastDone>0) { rotP=0; }                 // If you corrected recently OR the rotational correction isn't much, don't do it at all.
            // Add the speeds that it is trying to achieve to Smart Dashboard for debugging
            SmartDashboard.putNumber("rotP", rotP);

            if (!isRobotRelative)
                targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, -rotP, getGyroRotation()));   //Convert the desired speeds into individual wheel/module speeds. Radians
            else
                targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(speedX, speedY, rotP, getGyroRotation()));   //Convert the desired speeds into individual wheel/module speeds. Radians
            
            lastDone--;
        } else { // if turning dont proportional (don't correct the rotation based on the rotation 10 cycles ago)
            // need 
            
            if (!isRobotRelative)
                targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, -speedRot*0.8, getGyroRotation()));
            else
                targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(speedX, speedY, speedRot*0.8, getGyroRotation()));
            
            
                pointDir = getGyroRotation();

            lastDone = 10;
            
        }

        // Gets the target states from either {swerveDriveF} or {swerveDriveR} and applies them to individual modules (wheels)
        m_frontLeftModule .setTargetState(targetStates[0], true);
        m_frontRightModule.setTargetState(targetStates[1], true);
        m_backLeftModule  .setTargetState(targetStates[2], true);
        m_backRightModule .setTargetState(targetStates[3], true);


        m_frontLeftModule.go();
        m_frontRightModule.go();
        m_backLeftModule.go();
        m_backRightModule.go();
        

        //System.out.println("Back Left target/measure: "+targetStates[2].angle.getDegrees()+" | "+m_backLeftModule.getRotation().getDegrees());
        //System.out.println("Ecoders: "+m_frontLeftModule.getRotation().getDegrees()+", "+m_frontRightModule.getRotation().getDegrees()+", "+m_backLeftModule.getRotation().getDegrees()+", "+m_backRightModule.getRotation().getDegrees());
    }
    
    // drives
    // try applying acceleration cap to inputs in drives instead of on wheels

    // set how fast the swerve drive turns, rad/s allegedly
    public void setDriveRot(double sR, boolean squareInputs) {
        if (squareInputs) {
            sR=Math.signum(sR)*sR*sR;
        }
        speedRot = sR;

    }
    // set how fast the swerve drive goes, +x is forwards, +y is left and m/s hopefully 
    public void setDriveFXY(double sX, double sY, boolean squareInputs) {
        isRobotRelative = false;

        if (squareInputs) {
            double squareFactor = Math.sqrt(sX*sX+sY*sY);
            sX*=squareFactor;
            sY*=squareFactor;
        }
        
        speedX = sX;
        speedY = sY;
    }

    private int lastDone = 10;  // Cycles to sample rotation to make corrections to direction
    // Field Oriented swerve drive, m/s, m/s, rad/s or something, +x is forwards, +y is left
    public void swerveDriveF(double sX, double sY, double sR, boolean squareInputs) {
        isRobotRelative = false;

        SmartDashboard.putNumber("Gyro Target", pointDir.getDegrees());

        // input squaring, makes small adjustments easier while allowing higher speeds 
        if (squareInputs) {
            double squareFactor = Math.sqrt(sX*sX+sY*sY);
            sX*=squareFactor;
            sY*=squareFactor;
            sR=Math.signum(sR)*sR*sR;
        }
        
        speedX = sX;
        speedY = sY;
        speedRot = sR;
        
    }

    // Robot oriented swerve drive, m/s, m/s, rad/s
    // +speed is forwards, +strafe is left
    public void swerveDriveR(double speed, double strafe, double speedRot) {
        isRobotRelative = true;
        speedX = speed;
        speedY = -strafe;
        this.speedRot = speedRot;
        // targetStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(speed, -strafe, speedRot));
    }

    public void swerveDriveR(ChassisSpeeds newTargetStates) {
        isRobotRelative = true;
        double relativeSpeed = 1;
        speedX = newTargetStates.vxMetersPerSecond * relativeSpeed;
        speedY = -newTargetStates.vyMetersPerSecond * relativeSpeed;
        speedRot = -newTargetStates.omegaRadiansPerSecond;
        SmartDashboard.putNumber("Relative SpeedX", newTargetStates.vxMetersPerSecond);
        SmartDashboard.putNumber("Relative SpeedY", newTargetStates.vyMetersPerSecond);
        SmartDashboard.putNumber("Relative SpeedRot", newTargetStates.omegaRadiansPerSecond);
    }

    public boolean isOnRedAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.isPresent()) {
            return (ally.get() == Alliance.Red);
        }
        return false;
    }

    // car, m/s, degrees    
    public void carDrive(double speed, double turn) {
        turn = MathUtil.clamp(turn, -90, 90);
        
        // do the optimize thing
        // Gian: this 0.0174533 number is pi/180
        targetStates[0] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[1] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[2] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[3] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));

    }

    public void setTargetAngle(Rotation2d r) {
        pointDir = r;
    }
    
    // stuff
    // Degrees
    public double getGyroDegrees() {
        // Subtracted because the gyro was upside down meaning counter clockwise and clockwise were reversed...
        return getGyroRotation().getDegrees();
    }

    private final Rotation2d TWOPI = new Rotation2d(Math.PI*2); // TO BE REMOVED once the gyro is not upside down
    // Radians
    public Rotation2d getGyroRotation() {
        // Subtracted because the gyro was upside down meaning counter clockwise and clockwise were reversed...
        // return MathStuff.subtract(TWOPI, new Rotation2d(Units.degreesToRadians(m_gyro.getAngle())).minus(gyroOffset));
        
        // no longer subtracted because new gyro but idk | the number is pi/180
        return new Rotation2d(m_gyro.getAngle()*0.01745329251).minus(gyroOffset);
    }
    
    public Pose2d getPose() {
        return m_pose;
    }

    //for on the go field oriented and stuff
    public void zeroGyro() {
        pointDir = pointDir.minus(getGyroRotation());
        gyroOffset = gyroOffset.plus(getGyroRotation());
    }

    public void zeroGyro(Rotation2d offset) {
        pointDir = pointDir.minus(getGyroRotation().plus(offset));
        gyroOffset = gyroOffset.plus(getGyroRotation().plus(offset));
    }
    
    // Reset the expected position of the bot
    public void resetPose() {
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroRotation(), m_swervePositions);
        m_pose = new Pose2d();
    }

    // Reset the inputted pose
    // Only used by the holonomic builder
    public void resetPose(Pose2d pose2d) {
        pose2d = new Pose2d();
        // m_odometry.resetPosition(getGyroRotation(), m_swervePositions,m_pose);
        // m_pose = pose2d;
    }


    // This is mainly just for testing. This is what the wiki says to do.
    public Pose2d getPoseForPathPlanner() {
        // System.out.println(m_odometry.getPoseMeters());
        return m_pose;
        // return new Pose2d(-m_pose.getX(), m_pose.getY(), m_pose.getRotation());
        // return m_pose;
    }

    // This is mainly just for testing. This is what the wiki says to do.
    public void resetPoseForPathPlanner(Pose2d pose) {
        m_odometry.resetPosition(getGyroRotation(), m_swervePositions , m_pose);
        m_pose = pose;
    }

    // Returns the current robot-relative chasis speeds.
    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds cs = m_kinematics.toChassisSpeeds(m_swerveStates);
        // System.out.println("Speeds: " + cs);
        return cs;
    }

    public void stop() {
        m_frontLeftModule.setSpeeds(0, 0);
        m_frontRightModule.setSpeeds(0, 0);
        m_backLeftModule.setSpeeds(0, 0);
        m_backRightModule.setSpeeds(0, 0);
    }
    public void test() {
        
        //targetStates[0] = new SwerveModuleState(0.2, new Rotation2d());
        //targetStates[1] = new SwerveModuleState(0.2, new Rotation2d());
        //targetStates[2] = new SwerveModuleState(0.2, new Rotation2d());
        //targetStates[3] = new SwerveModuleState(0.2, new Rotation2d());
      //  
        //m_frontLeftModule .setTargetState(targetStates[0], false);
        //m_frontRightModule.setTargetState(targetStates[1], false);
        //m_backLeftModule  .setTargetState(targetStates[2], false);
        //m_backRightModule .setTargetState(targetStates[3], false);

        m_frontLeftModule.getSteerMotor().set(0.5);
        m_frontRightModule.getSteerMotor().set(0.5);
        m_backLeftModule.getSteerMotor().set(0.5);
        m_backRightModule.getSteerMotor().set(0.5);
    }

    public SwerveDriveKinematics getKinematics(){
        return m_kinematics;
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        return m_swervePositions;
    }
}