package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.ShootAuto;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;
import frc.robot.subsystems.swerve.ZeroGyro;

/**
 * <p> This does 4 things:
 * <p> 1) It primes the intake by getting it down to the ground.
 * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
 * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
 * <p> 4) It then moves back 1 meter.
 */
public class DiagonalOneNoteAuto extends SequentialCommandGroup {
    //subwoofer is 36.37in into field and ~41in hypotenuse, about 60 degree angle from front of field

    /**
     * <p> This starts by zeroing the robot such that the backwards direction is perpendicular to the alliance wall INSTEAD of being based on the robot's current rotation.
     * <p> It must be setup so that the robot is perfectly against/facing the speaker.
     * <p> This does 4 things:
     * <p> 1) It primes the intake by getting it down to the ground.
     * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
     * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
     * <p> 4) It then moves back 1 meter.
     * @param isRobotOnLeftSide When looking at the robot from the alliance wall, is the robot on the left of the speaker.
     */
    public DiagonalOneNoteAuto(boolean isRobotOnLeftSide) {
        super(
            // 120 offset on left side would make 0 degrees/forward be away from alliance wall
            new ZeroGyro(isRobotOnLeftSide ? -Constants.STARTING_ANGLE_DIAGONAL : Constants.STARTING_ANGLE_DIAGONAL),
            //new OneNoteAuto() cant because it zeros the gyro
            
            new ShootAuto(),
            new WaitCommand(11.5), // to not disturb any 4 note autos
            new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(2, 0)) // forwards should be away from alliance wall with gyro offset
        );
    }
}
