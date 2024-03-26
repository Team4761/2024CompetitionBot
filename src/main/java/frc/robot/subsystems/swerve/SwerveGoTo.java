package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.*;
import frc.robot.Constants;

/**
 * <p> This is a field-oriented move command.
 * <p> This uses swerve to move to a target odometry (it can be both at once)
 * <p> In this, the forwards direction is +x and the left direction is +y
 */
public class SwerveGoTo extends Command {
    protected SwerveDriveSubsystem m_swerve;
    protected Translation2d target;
    private double Pvalue = 0;
    //private double Ivalue = 0;
    private double maxVelocity = 4; //hard value
    private double vLimit = 0; // soft value based on acceleration cap

    private boolean isFinished = false;

    /**
     * <p> This initializes the command by setting the target position to target
     * <p> In this, +x is the forwards direction and +y is the left direction.
     * @param swerve A reference to the SwerveDriveSubsystem in the RobotMap to improve performance.
     * @param trans The position to go to from the 0,0 odometry
     */
    public SwerveGoTo(SwerveDriveSubsystem swerve, Translation2d target) {
        m_swerve = swerve;

        // commented because needs to be parallel with rotation
        //addRequirements(m_swerve);  // Make it so no 2 commands can access the swerve subsystem at the same time (first come first swerve)
        this.target = target;   // Set the target POSITION 
    }

    /**
     * <p> This initializes the command by setting the target position to the current position + the trans position and tells the SwerveSubsystem that it is needed.
     * <p> In this, +x is the forwards direction and +y is the left direction.
     * @param swerve A reference to the SwerveDriveSubsystem in the RobotMap to improve performance.
     * @param target The position to go to from the 0,0 odometry
     * @param speedLimit The max speed that the robot can get to as a number between 0.0 - ~4, in m/s
     */
    public SwerveGoTo(SwerveDriveSubsystem swerve, Translation2d target, double speedLimit) {
        m_swerve = swerve;
        //addRequirements(m_swerve);
        maxVelocity = speedLimit;
        this.target = target;   // Set the target POSITION 
    }

    /**
     * <p> While the command is running, the speed of the robot is determined by how far away the robot is from the desired location AND how long the command has run.
     * <p> The closer the robot gets to the target position, the slower it goes.
     * <p> The longer the command runs, the faster the robot goes.
     * <p> After finding the desired speed, it then uses the pythagorean theorum to limit the speed to {maxVelocity}
     * <p> This also checks whether or not the robot is within 1cm of the target position, and if it is, the command ends.
     */
    @Override
    public void execute() {
        Translation2d curTrans = m_swerve.getPose().getTranslation();   // Current translation travelled from 0,0
        
        // limit acceleration
        /* 
        double accelLimit = 4; //4 m/s^2
        double timeDifference = (System.currentTimeMillis()-lastTime)/1000;
        lastTime = System.currentTimeMillis();

        vLimit = Math.min(maxVelocity, vLimit+accelLimit*timeDifference); 
        accel limiter moved to swervedrivesubsystem
        */ 
        vLimit=maxVelocity;

        //correct forwards -y and left +y to actual speeds:
        double strafeGo = Constants.SWERVE_P * (target.getY()-curTrans.getY());  // The left/right speed where left = +y
        double speedGo = Constants.SWERVE_P * (target.getX()-curTrans.getX());    // The forwards/backwards speed where forwards = +x

        double hypoSpeed = Math.sqrt(strafeGo*strafeGo+speedGo*speedGo);    // Calculate the desired TOTAL speed (the hypotenus of the right triangle formed by the speed vectors)
        if (hypoSpeed>vLimit) { // If the desired speed is greater than the max speed, limit the strafe AND speed speed
            strafeGo = strafeGo/hypoSpeed*vLimit;
            speedGo = speedGo/hypoSpeed*vLimit;
        }

        SmartDashboard.putNumber("Auto Strafe Velocity", strafeGo);
        SmartDashboard.putNumber("Auto Foward Velocity", speedGo);
        SmartDashboard.putNumber("Auto PValue", Pvalue);
        SmartDashboard.putNumber("Distance To Target", target.getDistance(curTrans));

        isFinished = target.getDistance(curTrans) <= 0.05;  // If the distance is less than or equal to 10cm, then it is finished

        if (!isFinished)
            m_swerve.setDriveFXY(speedGo, strafeGo, false);        // Do field oriented swerve with the strafe and speed speeds
    }
    
    /**
     * <p> This ends the command if the robot is within 1cm of its desired position.
     * @return True if the position is within 1cm. False if the robot is further away than 1cm.
     */
    @Override
    public boolean isFinished() {
        
        return isFinished;  // Returns true if the distance between the robot and target is <= 1cm
    }
    
    /**
     * Once the command ends, it sets the speed of swerve to 0 so it doesn't just drift into oblivion.
     */
    @Override
    public void end(boolean interrupted) {
        m_swerve.setDriveFXY(0, 0, false); // Set all speeds to 0 AFTER close enough to end the command
    }
}