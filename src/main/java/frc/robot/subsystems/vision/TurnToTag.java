package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.MathStuff;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class TurnToTag extends Command {
    private SwerveDriveSubsystem m_swerve;
    private VisionSubsystem m_vision;
    private PhotonPipelineResult result;
    private Rotation2d targetYaw = null;
    
    private double Pvalue = 0;
    private double rotationDifference = 0;
    private Rotation2d lastRot;

    public TurnToTag(SwerveDriveSubsystem swerve, VisionSubsystem vision) {
        this.m_swerve = swerve;
        this.m_vision = vision;
    }
    @Override
    public void execute(){
        this.result = m_vision.getLatestResult();
        
        if(result.hasTargets()){
            //The yaw is in degrees fromthe result and changed to radians
            targetYaw = new Rotation2d(result.getBestTarget().getYaw());
        }
        else {
            // not setting to let it try to get to last seen tag
            // targetYaw = null;
        }
        //m_swerve.setDriveRot(rot, false);
        
        if (targetYaw != null) {



            Pvalue = MathUtil.clamp(-MathStuff.subtract(targetYaw, m_swerve.getGyroRotation()).getRotations()*Constants.SWERVE_ROTATE_P, -1.1, 1.1);
            //System.out.println(Pvalue);
            
            // make better subsystem support for this stuff
            m_swerve.setDriveRot(Pvalue, false);
            
            // degrees, to check if swerve is speeding past target or coming to a stop
            rotationDifference = Math.abs(m_swerve.getGyroRotation().minus(lastRot).getDegrees());
            lastRot = m_swerve.getGyroRotation();
        }
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(Pvalue)<0.033 && rotationDifference<0.15) { // && if robot moves <5 degrees/s in the past frame
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_swerve.setDriveRot(0,false);
    }
}
