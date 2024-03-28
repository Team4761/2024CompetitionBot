package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.FullIntake;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.shooter.RevShooter;
import frc.robot.subsystems.swerve.SwerveGoTo;
import frc.robot.subsystems.swerve.SwerveTurnTo;
/**
 * <p> The robot must be placed so that when the intake drops, it falls directly on top of a note.
 * <p>
 * <p> This does 8 things:
 * <p> 1) It primes the intake by getting it down to the ground.
 * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
 * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
 * <p> 4) It runs the bottom and top intake to get the note to the actual shooter, changing the shooter's angle in the process.
 * <p> 5) It gets the shooter back to the correct angle, and then shoots.
 * <p> 6) It uses a series of move commands to get to the top note in the alliance side of notes.
 * <p> 7) It full intakes the note while turning 33 degrees to face the speaker to shoot.
 * <p> 8) It revs up the shooter and then shoots.
 * <p> 9) It backs up 1 meter.
 */
public class ThreeNoteAuto extends SequentialCommandGroup {

    /**
     * <p> The robot must be placed so that when the intake drops, it falls directly on top of a note.
     * <p>
     * <p> This does 8 things:
     * <p> 1) It primes the intake by getting it down to the ground.
     * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
     * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
     * <p> 4) It runs the bottom and top intake to get the note to the actual shooter, changing the shooter's angle in the process.
     * <p> 5) It gets the shooter back to the correct angle, and then shoots.
     * <p> 6) It uses a series of move commands to get to the top note in the alliance side of notes.
     * <p> 7) It full intakes the note while turning 33 degrees to face the speaker to shoot.
     * <p> 8) It revs up the shooter and then shoots.
     * <p> 9) It backs up 1 meter.
     */
    public ThreeNoteAuto(boolean blueAlliance) { // do flipping stuff. also on 4 note, also measure swerve width/length/wheel diameter

        // notes are placed with a 57 inch offset left/right, 1.45ish meters
        super(
            new TwoNoteAuto(), // shoot start note and center note, should be at 0,0 at the end

            // go to and intake left(when looking at alliance wall) note
            new ParallelCommandGroup(
                new GetShooterToAngle(Constants.SHOOTER_INTAKE_ANGLE),
                
                new ParallelDeadlineGroup( // doesnt need to drive if intaked
                    new SequentialCommandGroup(
                        new WaitCommand(0.1),
                        new SwerveGoTo(Robot.getMap().swerve, new Translation2d(1.55, blueAlliance ? 1.55 : -1.55)) // rotate during this
                    ),
                    new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(blueAlliance ? 0.8*Math.PI : -0.8*Math.PI)),
                    new RunIntake(0.7, 3000)
                    //new IntakeUntilBreakbeam(4000) 
                    
                )
            ),

            // intake and go back to 0,0
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new FullIntake(Constants.AUTO_INTAKE_SPEED, Constants.AUTO_UPTAKE_SPEED, new Rotation2d(Constants.SHOOTER_SHOOT_ANGLE)).withTimeout(4),
                    new SwerveGoTo(Robot.getMap().swerve, new Translation2d(0.25, 0)),
                    new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(Math.PI))
                ),
                new SequentialCommandGroup(
                    new WaitCommand(0.8), // trying not to run shooter when also intaking and accelerating swerve
                    new RevShooter(40) // 
                )
            ),

            // get shooter to angle and rev
            new ParallelDeadlineGroup(
                new GetShooterToAngle(Constants.SHOOTER_SHOOT_ANGLE),      // Get the shooter to shooting position
                new RevShooter(40, 4) // rev motor while shooter is angling
            ),

            // shoot
            new IntakeAndShoot(40, 0) // change rev to go parallel with shooterangle
        );
    }
}
