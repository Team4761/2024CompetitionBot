package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

// field oriented, move relative to current position, forward should be -y, left +x
public class SwerveGoCartesianF extends Command {
    private SwerveDriveSubsystem m_swerve;
    private Translation2d target;
    private double Pvalue = 0;
    private double vLimit = 0.7;

    // trans contains an x and y component for the desired movement (NOT new position). +x is fowards, +y is left.
    public SwerveGoCartesianF(SwerveDriveSubsystem swerve, Translation2d trans) {
        m_swerve = swerve;
        addRequirements(m_swerve);  // Make it so no 2 commands can access the swerve subsystem at the same time (first come first swerve)
        target = m_swerve.getPose().getTranslation().plus(trans);   // Set the target POSITION (not translation)
    }

    // Same as the method above but with a limiter for the speed where speedlimit is between 0.0-1.0 i think
    public SwerveGoCartesianF(SwerveDriveSubsystem swerve, Translation2d trans, double speedLimit) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        target = m_swerve.getPose().getTranslation().plus(trans);
        vLimit = speedLimit;
    }

    
    @Override
    public void initialize() {
        
    }


    @Override
    public void execute() {
        Translation2d curTrans = m_swerve.getPose().getTranslation();   // Current translation travelled from 0,0
        // adjust pid off units
        Pvalue = Math.min(target.getDistance(curTrans)*0.25, 0.2);    // As you get closer to the target, you slow down.

        //correct forwards -y and left +y to 
        double strafeGo = -Pvalue*(target.getY()-curTrans.getY());  // The left/right speed where left = +y
        double speedGo = Pvalue*(target.getX()-curTrans.getX());    // The forwards/backwards speed where forwards = +x

        double hypoSpeed = Math.sqrt(strafeGo*strafeGo+speedGo*speedGo);    // Calculate the desired TOTAL speed (the hypotenus of the right triangle formed by the speed vectors)
        if (hypoSpeed>vLimit) { // If the desired speed is greater than the max speed, limit the strafe AND speed speed
            strafeGo = strafeGo/hypoSpeed*vLimit;
            speedGo = speedGo/hypoSpeed*vLimit;
        }


        m_swerve.swerveDriveF(strafeGo, speedGo, 0);        // Do field oriented swerve with the strafe and speed speeds
    }
    
    @Override
    public boolean isFinished() {
        if(Pvalue<0.01) return true;    // If close to the desired translation (position), then finish the command.
        else return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerve.swerveDriveF(0, 0, 0); // Set all speeds to 0 AFTER close enough to end the command
    }
}