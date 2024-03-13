package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.IntakeAndLaunchPiece;
import frc.robot.Auto.IntakeLaunchMove;
import frc.robot.Auto.MoveBackCommand;
import frc.robot.subsystems.intake.GetIntakeToSetPosition;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.shooter.Shoot;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;

/**
     * <h1> This is the best auto.
     * <p> This auto's only purpose is to go to the center and steal the notes from the center of the field.
     * <p> The robot must start with the intake facing the center and the robot on the alliance line directly behind the leftmost or rightmost note.
     * <p> This auto does the following actions:
     * <p>
     * <p> 1) It drops the intake and shooter. The shooter will be in Intake position
     * <p> 2) It lightly shoots the note out that it was loaded with WHILE moving back (it does them at the same time)
     * <p> 3) Once it gets to the center, it starts the intake for 1 second to pick up a piece.
     * <p> 4) It moves the piece through the entire robot while lightly shooting it WHILE moving left/right to the next piece.
     * <p> 5) Repeat steps 3 and 4 for the remaining 4 pieces.
     */
public class MessEmUpAuto extends SequentialCommandGroup {
    
    private final static double DISTANCE_BETWEEN_NOTES = 0.3;          // The distance between the center of the notes in the center of the field
    private final static double DISTANCE_FROM_LINE_TO_NOTES = 1.0;     // The distance between the alliance line and the edge of the first center note.

    /**
     * <h1> This is the best auto.
     * <p> This auto's only purpose is to go to the center and steal the notes from the center of the field.
     * <p> The robot must start with the intake facing the center and the robot on the alliance line directly behind the leftmost or rightmost note.
     * <p> This auto does the following actions:
     * <p>
     * <p> 1) It drops the intake and shooter. The shooter will be in Intake position
     * <p> 2) It lightly shoots the note out that it was loaded with WHILE moving back (it does them at the same time)
     * <p> 3) Once it gets to the center, it starts the intake for 1 second to pick up a piece.
     * <p> 4) It moves the piece through the entire robot while lightly shooting it WHILE moving left/right to the next piece.
     * <p> 5) Repeat steps 3 and 4 for the remaining 4 pieces. 
     * @param shouldGoLeft Looking at your robot from your alliance side, once the robot gets to the center, should it go left or right?
     */
    public MessEmUpAuto(boolean shouldGoLeft) {
        super(
            new SequentialCommandGroup(                                                                     // Get the intake down first and then the shooter up.
                // new GetIntakeToSetPosition(Constants.INTAKE_INTAKE_POSITION),                               // Get the intake to the down position
                new GetShooterToAngle(Constants.SHOOTER_INTAKE_ANGLE)                                       // Get the shooter to connect with the intake
            ),
            new ParallelCommandGroup(                                                                       // Move back while lightly shooting the piece out
                new IntakeAndShoot(10),                                                               // Lightly shoot the piece
                new MoveBackCommand(DISTANCE_FROM_LINE_TO_NOTES)                                            // Move back WIP meters
            ),
            new IntakeLaunchMove(shouldGoLeft, DISTANCE_BETWEEN_NOTES),                                     // Full intake the piece, shoot it out, and move left or right
            new IntakeLaunchMove(shouldGoLeft, DISTANCE_BETWEEN_NOTES),                                     // Full intake the piece, shoot it out, and move left or right
            new IntakeLaunchMove(shouldGoLeft, DISTANCE_BETWEEN_NOTES),                                     // Full intake the piece, shoot it out, and move left or right
            new IntakeLaunchMove(shouldGoLeft, DISTANCE_BETWEEN_NOTES),                                     // Full intake the piece, shoot it out, and move left or right
            new RunIntake(Robot.getShuffleboard().getSettingNum("Intake Speed"), 1)            // End by intaking
        );
    }



    
}
