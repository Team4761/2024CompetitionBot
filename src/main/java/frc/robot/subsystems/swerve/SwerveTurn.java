package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

// turn x degrees, not to a gyro degree
public class SwerveTurn extends Command {
    private SwerveDriveSubsystem m_swerve;
    private Rotation2d target;

    public SwerveTurn(SwerveDriveSubsystem swerve, Rotation2d rot) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        target = rot;
    }

    
    @Override
    public void initialize() {
        m_swerve.setTargetAngle(m_swerve.getGyroRotation().plus(target));
    }

    @Override
    public void execute() {
        
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public void end(boolean interrupted) {

    }
}
