package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

// turn x degrees, not to a gyro degree
public class SwerveTurnTo extends Command {
    private SwerveDriveSubsystem m_swerve;
    private Rotation2d target;

    private double Pvalue = 0;

    public SwerveTurnTo(SwerveDriveSubsystem swerve, Rotation2d rot) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        target = rot;
    }

    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Pvalue = Math.min(Math.max(MathStuff.subtract(target, m_swerve.getGyroRotation()).getRotations()*30, -0.9), 0.9);
        //System.out.println(Pvalue);
        
        // make better subsystem support for this stuff
        m_swerve.setDriveRot(Pvalue, false);
        
    }
    
    @Override
    public boolean isFinished() {
        System.out.println(Pvalue);
        if(Pvalue<0.01) {
            System.out.println("DONEONODNONE");
            return true;
        }
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerve.setTargetAngle(target);
        m_swerve.setDriveRot(0, false);
    }
}
