package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class Move extends Command {

    private SwerveDriveSubsystem m_swerve;
    private Translation2d target;
    private double Pvalue = 0;
    private double Ivalue = 0;
    private double vLimit = 1;

    private boolean isFinished = false;
    
    /**
     * <p> Makes the robot move a distance x and y
     * @param x Meters. + is forwards, - is backwards.
     * @param y Meters. + is left, - is backwards.
     */
    public Move(double x, double y) {
        m_swerve = Robot.getMap().swerve;
        addRequirements(m_swerve);  // Make it so no 2 commands can access the swerve subsystem at the same time (first come first swerve)
        target = m_swerve.getPose().getTranslation().plus(new Translation2d(x, y));   // Set the target POSITION (not translation)
    }


    @Override
    public void execute() {
        Translation2d curTrans = m_swerve.getPose().getTranslation();   // Current translation travelled from 0,0
        // adjust pid off units
        Pvalue = Math.min(target.getDistance(curTrans)*Constants.SWERVE_P, 0.7);    // As you get closer to the target, you slow down.
        Ivalue += Constants.SWERVE_I / 60.0;   // The speed (Ivalue) builds up over time

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

        isFinished = target.getDistance(curTrans) <= 0.01;  // If the distance is less than or equal to 1cm, then it is finished

        if (!isFinished)
            m_swerve.swerveDriveF(speedGo, strafeGo, 0, false);        // Do field oriented swerve with the strafe and speed speeds
    }
}
