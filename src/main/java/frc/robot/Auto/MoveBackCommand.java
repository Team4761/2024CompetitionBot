package frc.robot.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;

public class MoveBackCommand extends SequentialCommandGroup{
    
    /**
     * Moves the robot back a certain distance
     * @param distance In meters
     */
    public MoveBackCommand(double distance){
        super(
            new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(-distance, 0)),
            new PrintCommand("Finished Move Back!")
        );
    }
}
