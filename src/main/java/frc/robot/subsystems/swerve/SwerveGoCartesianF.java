package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * <p> This is a field-oriented move command.
 * <p> This uses swerve to move by an offset in the x and y direction (it can be both at once)
 * <p> In this, the forwards direction is +x and the left direction is +y
 */
public class SwerveGoCartesianF extends SwerveGoTo {
    private Translation2d trans;

    /**
     * <p> This initializes the command by setting the target position to the current position + the trans position and tells the SwerveSubsystem that it is needed.
     * <p> In this, +x is the forwards direction and +y is the left direction.
     * @param swerve A reference to the SwerveDriveSubsystem in the RobotMap to improve performance.
     * @param trans The offset from the current position. Example, Translation2d(1, 0) would move the robot forwards 1 meter regardless of where it currently is.
     */
    public SwerveGoCartesianF(SwerveDriveSubsystem swerve, Translation2d trans) {
        super(swerve, new Translation2d());

        //addRequirements(m_swerve);  // Make it so no 2 commands can access the swerve subsystem at the same time (first come first swerve)
        this.trans = trans; 
    }

    /**
     * <p> This initializes the command by setting the target position to the current position + the trans position and tells the SwerveSubsystem that it is needed.
     * <p> In this, +x is the forwards direction and +y is the left direction.
     * @param swerve A reference to the SwerveDriveSubsystem in the RobotMap to improve performance.
     * @param trans The offset from the current position. Example, Translation2d(1, 0) would move the robot forwards 1 meter regardless of where it currently is.
     * @param speedLimit The max speed that the robot can get to as a number between 0.0 - 1.0 as 0% speed to 100% speed.
     */
    public SwerveGoCartesianF(SwerveDriveSubsystem swerve, Translation2d trans, double speedLimit) {
        super(swerve, new Translation2d(), speedLimit);
        
        //addRequirements(m_swerve);  // Make it so no 2 commands can access the swerve subsystem at the same time (first come first swerve)
        this.trans = trans; 
    }

    public void initialize() {
        super.target = m_swerve.getPose().getTranslation().plus(trans);
        super.lastTime = System.currentTimeMillis();
    }
}