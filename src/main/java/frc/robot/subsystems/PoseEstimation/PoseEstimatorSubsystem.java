package frc.robot.subsystems.PoseEstimation;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class PoseEstimatorSubsystem extends SubsystemBase{
    
    //Necessary for estimator
    private RobotMap map = Robot.getMap();
    private Pose2d pose = new Pose2d();
    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] positions;
    private Rotation2d gyroRotation;
    private SwerveDriveSubsystem swerve = map.swerve;
    private VisionSubsystem vision = map.vision;
    private Pose2d visionPose2d = new Pose2d();
    private Pose2d prevVisionPose2d = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();

    private SwerveDrivePoseEstimator poseEstimator;
    
    public PoseEstimatorSubsystem(){
        //Initialization
        this.kinematics = swerve.getKinematics();
        this.positions = swerve.getSwerveModulePositions();
        this.gyroRotation = swerve.getGyroRotation();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroRotation, positions, pose);
    }
    
    public void periodic(){
        //Checks if vision poses are the same if they are update estimator w/o vision data 
        //If tag is detected and yields good results then add vision to estimator w gyro and positions
        prevVisionPose2d = visionPose2d;
        var result = vision.getLatestResult();
        if(result.hasTargets()){
            if(result.getBestTarget().getPoseAmbiguity() < .2){
                visionPose2d = vision.getVisionPose();
            }
        }

        this.gyroRotation = swerve.getGyroRotation();
        this.positions = swerve.getSwerveModulePositions();

        if(!prevVisionPose2d.equals(visionPose2d)){
            poseEstimator.addVisionMeasurement(visionPose2d, result.getTimestampSeconds());
        }

        estimatedPose = poseEstimator.update(gyroRotation, positions);
    }

    public Pose2d getEstimatedPose(){
        return estimatedPose;
    }
}
