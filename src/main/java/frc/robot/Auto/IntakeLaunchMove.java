package frc.robot.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;

/**
 * <p> This fully intakes the note, launches it as though it was going for amp, and moves left/right
 */
public class IntakeLaunchMove extends SequentialCommandGroup {

    public IntakeLaunchMove(boolean shouldGoLeft, double distanceBetweenNotes) {
        new SequentialCommandGroup(                                                                         // Intake and then move right/left while getting rid of the piece
            new RunIntake(Robot.getShuffleboard().getSettingNum("Intake Speed"), 1),           // Run the intake for a second
            new ParallelCommandGroup(                                                                       // Move WHILE full intaking and shooting the piece out
                new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(0, shouldGoLeft ? distanceBetweenNotes : -distanceBetweenNotes)),// Move to the next note
                new IntakeAndLaunchPiece(                                                                   // Fully intake the piece and launch it
                    Robot.getShuffleboard().getSettingNum("Intake Speed"),                              // Get the actual intake speed
                    Robot.getShuffleboard().getSettingNum("Amp Shoot Speed"))                           // Shoot as though it was shooting amp
            )
        );
    }
}
