package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * <p> This is a field-oriented move command.
 * <p> This uses swerve to move to a target odometry (it can be both at once)
 * <p> In this, the forwards direction is +x and the left direction is +y
 */
public class SwerveGoThrough extends Command {
    protected SwerveDriveSubsystem m_swerve;
    protected Translation2d target;
    private double r = 0.1; //0.1 m default
    //private double Ivalue = 0;
    private double maxVelocity = 4; //hard value

    private boolean isFinished = false;

    /**
     * <p> This initializes the command by setting the target position to target
     * <p> In this, +x is the forwards direction and +y is the left direction.
     * @param swerve A reference to the SwerveDriveSubsystem in the RobotMap to improve performance.
     * @param trans The position to go to from the 0,0 odometry
     */
    public SwerveGoThrough(SwerveDriveSubsystem swerve, Translation2d target) {
        m_swerve = swerve;
        this.target = target;   // Set the target POSITION 
    }

    /**
     * <p> This initializes the command by setting the target position to the current position + the trans position and tells the SwerveSubsystem that it is needed.
     * <p> In this, +x is the forwards direction and +y is the left direction.
     * @param swerve A reference to the SwerveDriveSubsystem in the RobotMap to improve performance.
     * @param target The position to go to from the 0,0 odometry
     * @param speed The speed to go out, m/s
     */
    public SwerveGoThrough(SwerveDriveSubsystem swerve, Translation2d target, double speed) {
        this(swerve, target);
        maxVelocity = speed;
    }

    // set tolerance for finish (meters)
    public SwerveGoThrough(SwerveDriveSubsystem swerve, Translation2d target, double speed, double radius) {
        this(swerve, target, speed);
        r=radius;
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
        

        Translation2d direction = new Translation2d(target.getX()-curTrans.getX(), target.getY()-curTrans.getY()); // direction to go in
        direction=direction.div(direction.getNorm()); //normalize the direction
        direction=direction.times(maxVelocity); // assumes can go max velocity


        isFinished = target.getDistance(curTrans) <= r;  // If the distance is less than or equal to radius, then it is finished

        if (!isFinished)
            m_swerve.setDriveFXY(direction.getX(), direction.getY(), false);        // Do field oriented swerve with the strafe and speed speeds
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
        // m_swerve.setDriveFXY(0, 0, false); // do nothing afterwards, assumes a goto or similar command will happen after
    }
}