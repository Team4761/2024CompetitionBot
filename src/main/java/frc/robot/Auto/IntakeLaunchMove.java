package frc.robot.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.swerve.Move;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;

/**
 * <p> This fully intakes the note, launches it as though it was going for amp, and moves left/right
 */
public class IntakeLaunchMove extends SequentialCommandGroup {

    public IntakeLaunchMove(boolean shouldGoLeft, double distanceBetweenNotes) {
        super(                                                                       // Intake and then move right/left while getting rid of the piece
            new PrintCommand("Started IntakeLaunchMove"),
            new RunIntake(Robot.getShuffleboard().getSettingNum("Intake Speed"), 1000),        // Run the intake for a second
            new PrintCommand("Finished Intake!"),
            new ParallelCommandGroup(                                                                       // Move WHILE full intaking and shooting the piece out
                new Move(0, shouldGoLeft ? -distanceBetweenNotes : distanceBetweenNotes),// Move to the next note
                new IntakeAndLaunchPiece(                                                                   // Fully intake the piece and launch it
                    Robot.getShuffleboard().getSettingNum("Intake Speed"),                              // Get the actual intake speed
                    Robot.getShuffleboard().getSettingNum("Amp Shoot Speed"))                           // Shoot as though it was shooting amp
            ),
            new PrintCommand("Finished Go Left!")
        );
    }
}
