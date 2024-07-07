package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DistanceToTag extends Command{
    private SwerveDriveSubsystem swerve;
    private VisionSubsystem vision;
    private double actualDistance;
    private double desiredDistance;
    private PhotonPipelineResult result;
    private double speed;
    private double cameraPitch = Units.degreesToRadians(15);
    private PIDController controller;

    public DistanceToTag(SwerveDriveSubsystem swerve, VisionSubsystem vision, double distance){
        this.swerve = swerve;
        this.vision = vision;
        this.desiredDistance = distance;
        this.controller = new PIDController(0,0,0);
    }

    @Override
    public void execute(){
        this.result = vision.getLatestResult();
        if(result.hasTargets()){
            //Pitch is returned in degrees changed to radians
            double targetPitch = Units.degreesToRadians(result.getBestTarget().getPitch());
            double pitch = cameraPitch + targetPitch;
            
            int tagID = result.getBestTarget().getFiducialId();
            Pose3d tagPose = vision.getTagPose(tagID);
            double targetHeight = tagPose.getZ();

            actualDistance = targetHeight/Math.tan(pitch);

            speed = controller.calculate(actualDistance, desiredDistance);
        }   
        else{
            speed = 0;
        }
        swerve.swerveDriveF(speed,0,0, false);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(actualDistance - desiredDistance) < .25){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        swerve.swerveDriveF(0,0,0, false);
    }

}
