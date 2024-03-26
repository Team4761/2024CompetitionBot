package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * <p> This turns a specific number of degrees.
 * <p> This does NOT turn to a specific rotation, this turns an offset from the current rotation.
 */
public class SwerveTurn extends SwerveTurnTo {
    private Rotation2d rotation;

    /**
     * This initializes the internal storage of the variables as well as tells the robot that the SwerveSubsystem is needed.
     * @param swerve Reference to the RobotMap's SwerveDriveSubsystem to improve performance.
     * @param rot The desired rotation to turn the robot by. Once again, this is NOT the desired final rotation, this is just an offset from the current rotation.
     */
    public SwerveTurn(SwerveDriveSubsystem swerve, Rotation2d rot) {
        super(swerve, rot);
        rotation = rot;
    }

    @Override
    public void initialize() {
        super.initialize();
        super.target = m_swerve.getGyroRotation().plus(rotation);
    }
}
