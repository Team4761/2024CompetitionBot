package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * <p> This turns a specific number of degrees.
 * <p> This does NOT turn to a specific rotation, this turns an offset from the current rotation.
 */
public class SwerveTurnTo extends Command {
    private SwerveDriveSubsystemInterface m_swerve;
    private Rotation2d target;

    private double Pvalue = 0;

    /**
     * This initializes the internal storage of the variables as well as tells the robot that the SwerveSubsystem is needed.
     * @param swerve Reference to the RobotMap's SwerveDriveSubsystem to improve performance.
     * @param rot The desired rotation to turn the robot to by in degrees/radians (it's a Rotation2d so it doesn't matter).
     */
    public SwerveTurnTo(SwerveDriveSubsystemInterface swerve, Rotation2d rot) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        target = rot;
    }

    /**
     * <p> This finds the differnce between the target rotation and the current rotation and then uses that to set the speed.
     * <p> This means that the closer the robot gets to the desired rotation, the slower it will go.
     */
    @Override
    public void execute() {
        Pvalue = Math.min(Math.max(MathStuff.subtract(target, m_swerve.getGyroRotation()).getRotations()*30, -0.9), 0.9);
        //System.out.println(Pvalue);
        
        // make better subsystem support for this stuff
        m_swerve.setDriveRot(Pvalue, false);
        
    }
    
    /**
     * <p> This checks for if the speed that the robot will be set to rotate is less than 0.01 radians / second.
     * @return True if the speed is slow (which means the robot is very close to the desired rotation). False if the speed is fast.
     */
    @Override
    public boolean isFinished() {
        System.out.println(Pvalue);
        if(Pvalue<0.01) {
            System.out.println("DONEONODNONE");
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
