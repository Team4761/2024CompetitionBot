package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

// turn x degrees, not to a gyro degree
public class SwerveTurnDegrees extends Command {
    private SwerveDriveSubsystem m_swerve;
    private Rotation2d target;

    public SwerveTurnDegrees(SwerveDriveSubsystem swerve, Rotation2d rot) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        target = rot;
    }

    
    @Override
    public void initialize() {
        
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
