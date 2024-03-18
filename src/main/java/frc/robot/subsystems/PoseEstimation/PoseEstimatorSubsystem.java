package frc.robot.subsystems.PoseEstimation;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class PoseEstimatorSubsystem extends SubsystemBase{
    
    //Necessary for estimator 
    private Pose2d visionPose2d = new Pose2d();
    private Pose2d prevVisionPose2d = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();

    private SwerveDrivePoseEstimator poseEstimator;
    
    public PoseEstimatorSubsystem(){
        poseEstimator = new SwerveDrivePoseEstimator(Robot.getMap().swerve.getKinematics(), Robot.getMap().swerve.getGyroRotation(), Robot.getMap().swerve.getSwerveModulePositions(), Robot.getMap().swerve.getPose());
    }
    
    public void periodic(){
        //Checks if vision poses are the same if they are update estimator w/o vision data 
        //If tag is detected and yields good results then add vision to estimator w gyro and positions
        prevVisionPose2d = visionPose2d;
        var result = Robot.getMap().vision.getLatestResult();
        if(result.hasTargets()){
            if(result.getBestTarget().getPoseAmbiguity() < .2){
                visionPose2d = Robot.getMap().vision.getVisionPose();
            }
        }
        if(!prevVisionPose2d.equals(visionPose2d)){
            poseEstimator.addVisionMeasurement(visionPose2d, result.getTimestampSeconds());
        }
        estimatedPose = poseEstimator.update(Robot.getMap().swerve.getGyroRotation(), Robot.getMap().swerve.getSwerveModulePositions());
    }

    public Pose2d getEstimatedPose(){
        return estimatedPose;
    }
}
