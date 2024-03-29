package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * <p> This turns a specific number of degrees.
 * <p> This does NOT turn to a specific rotation, this turns an offset from the current rotation.
 */
public class SwerveTurnTo extends Command {
    protected SwerveDriveSubsystem m_swerve;
    protected Rotation2d target;

    private double Pvalue = 0;
    private double rotationDifference = 0;
    private Rotation2d lastRot;

    /**
     * This initializes the internal storage of the variables as well as tells the robot that the SwerveSubsystem is needed.
     * @param swerve Reference to the RobotMap's SwerveDriveSubsystem to improve performance.
     * @param rot The desired rotation to turn the robot to by in degrees/radians (it's a Rotation2d so it doesn't matter).
     */
    public SwerveTurnTo(SwerveDriveSubsystem swerve, Rotation2d rot) {
        m_swerve = swerve;
        // addRequirements(m_swerve); runs in parallel with cartesian
        target = rot;
    }

    @Override
    public void initialize() {
        lastRot = m_swerve.getGyroRotation();
    }

    /**
     * <p> This finds the differnce between the target rotation and the current rotation and then uses that to set the speed.
     * <p> This means that the closer the robot gets to the desired rotation, the slower it will go.
     */
    @Override
    public void execute() {
        Pvalue = MathUtil.clamp(-MathStuff.subtract(target, m_swerve.getGyroRotation()).getRotations()*Constants.SWERVE_ROTATE_P, -1.1, 1.1);
        //System.out.println(Pvalue);
        
        // make better subsystem support for this stuff
        m_swerve.setDriveRot(Pvalue, false);
        
        // degrees, to check if swerve is speeding past target or coming to a stop
        rotationDifference = Math.abs(m_swerve.getGyroRotation().minus(lastRot).getDegrees());
        lastRot = m_swerve.getGyroRotation();
    }
    
    /**
     * <p> This checks for if the speed that the robot will be set to rotate is less than 0.01 radians / second.
     * @return True if the speed is slow (which means the robot is very close to the desired rotation). False if the speed is fast.
     */
    @Override
    public boolean isFinished() {
        if(Math.abs(Pvalue)<0.033 && rotationDifference<0.15) { // && if robot moves <5 degrees/s in the past frame
            return true;
        }
        return false;
    }
    
    /**
     * <p> After getting to the desired rotation, tell swerve that it should be at the {target} rotation and set its speed to 0 to stop it from moving.
     */
    @Override
    public void end(boolean interrupted) {
        m_swerve.setTargetAngle(target);
        m_swerve.setDriveRot(0, false);
    }
}
