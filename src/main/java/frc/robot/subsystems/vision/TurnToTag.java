package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class TurnToTag extends Command {
    private SwerveDriveSubsystem swerve;
    private VisionSubsystem vision;
    private PhotonPipelineResult result;
    private double targetYaw;
    private double setPoint = 0;
    private double rot = 0;
    private PIDController controller;

    public TurnToTag(SwerveDriveSubsystem swerve, VisionSubsystem vision){
        this.swerve = swerve;
        this.vision = vision;
        controller = new PIDController(3,0,0.01);
    }
    @Override
    public void execute(){
        this.result = vision.getLatestResult();
        if(result.hasTargets()){
            //The yaw is in degrees fromthe result and changed to radians
            targetYaw = Units.degreesToRadians(result.getBestTarget().getYaw());
            rot = -controller.calculate(targetYaw, setPoint);
        }
        else{
            rot = 0;
            targetYaw = 0;
        }
        swerve.setDriveRot(rot, false);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(targetYaw - setPoint) < 5){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        swerve.setDriveRot(0,false);
    }
}
