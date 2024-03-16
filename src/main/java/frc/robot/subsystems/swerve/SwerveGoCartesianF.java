package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/**
 * <p> This is a field-oriented move command.
 * <p> This uses swerve to move by an offset in the x and y direction (it can be both at once)
 * <p> In this, the forwards direction is +x and the left direction is +y
 */
public class SwerveGoCartesianF extends Command {
    private SwerveDriveSubsystem m_swerve;
    private Translation2d target;
    private double Pvalue = 0;
    private double Ivalue = 0;
    private double vLimit = 0.7;

    private boolean isFinished = false;

    /**
     * <p> This initializes the command by setting the target position to the current position + the trans position and tells the SwerveSubsystem that it is needed.
     * <p> In this, +x is the forwards direction and +y is the left direction.
     * @param swerve A reference to the SwerveDriveSubsystem in the RobotMap to improve performance.
     * @param trans The offset from the current position. Example, Translation2d(1, 0) would move the robot forwards 1 meter regardless of where it currently is.
     */
    public SwerveGoCartesianF(SwerveDriveSubsystem swerve, Translation2d trans) {
        m_swerve = swerve;
        addRequirements(m_swerve);  // Make it so no 2 commands can access the swerve subsystem at the same time (first come first swerve)
        target = m_swerve.getPose().getTranslation().plus(trans);   // Set the target POSITION (not translation)
    }

    /**
     * <p> This initializes the command by setting the target position to the current position + the trans position and tells the SwerveSubsystem that it is needed.
     * <p> In this, +x is the forwards direction and +y is the left direction.
     * @param swerve A reference to the SwerveDriveSubsystem in the RobotMap to improve performance.
     * @param trans The offset from the current position. Example, Translation2d(1, 0) would move the robot forwards 1 meter regardless of where it currently is.
     * @param speedLimit The max speed that the robot can get to as a number between 0.0 - 1.0 as 0% speed to 100% speed.
     */
    public SwerveGoCartesianF(SwerveDriveSubsystem swerve, Translation2d trans, double speedLimit) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        target = m_swerve.getPose().getTranslation().plus(trans);
        vLimit = speedLimit;
    }



    /**
     * <p> While the command is running, the speed of the robot is determined by how far away the robot is from the desired location AND how long the command has run.
     * <p> The closer the robot gets to the target position, the slower it goes.
     * <p> The longer the command runs, the faster the robot goes.
     * <p> After finding the desired speed, it then uses the pythagorean theorum to limit the speed to {vLimit}
     * <p> This also checks whether or not the robot is within 1cm of the target position, and if it is, the command ends.
     */
    @Override
    public void execute() {
        Translation2d curTrans = m_swerve.getPose().getTranslation();   // Current translation travelled from 0,0
        // adjust pid off units
        Pvalue = Math.min(target.getDistance(curTrans)*Constants.SWERVE_P, 0.5);    // As you get closer to the target, you slow down.
        Ivalue += Constants.SWERVE_I / 100.0;   // The speed (Ivalue) builds up over time

        //correct forwards -y and left +y to actual speeds:
        double strafeGo = Math.min(1,Ivalue) * Pvalue * (target.getY()-curTrans.getY());  // The left/right speed where left = +y
        double speedGo = Math.min(1,Ivalue) * Pvalue * (target.getX()-curTrans.getX());    // The forwards/backwards speed where forwards = +x

        double hypoSpeed = Math.sqrt(strafeGo*strafeGo+speedGo*speedGo);    // Calculate the desired TOTAL speed (the hypotenus of the right triangle formed by the speed vectors)
        if (hypoSpeed>vLimit) { // If the desired speed is greater than the max speed, limit the strafe AND speed speed
            strafeGo = strafeGo/hypoSpeed*vLimit;
            speedGo = speedGo/hypoSpeed*vLimit;
        }
        SmartDashboard.putNumber("Auto Strafe Velocity", strafeGo);
        SmartDashboard.putNumber("Auto Foward Velocity", speedGo);
        SmartDashboard.putNumber("Auto PValue", Pvalue);
        SmartDashboard.putNumber("Distance To Target", target.getDistance(curTrans));

        isFinished = target.getDistance(curTrans) <= 0.1;  // If the distance is less than or equal to 1cm, then it is finished

        if (!isFinished)
            m_swerve.swerveDriveF(speedGo, strafeGo, 0, false);        // Do field oriented swerve with the strafe and speed speeds
    }
    
    /**
     * <p> This ends the command if the robot is within 1cm of its desired position.
     * @return True if the position is within 1cm. False if the robot is further away than 1cm.
     */
    @Override
    public boolean isFinished() {
        // if(Pvalue<0.01) return true;    // If close to the desired translation (position), then finish the command.
        // else return false;
        if (isFinished)
            m_swerve.setDriveFXY(0, 0, false);  // This exists to force the current code to autocorrect its rotational position at the end. (because the code is iffy)
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