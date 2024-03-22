package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.ShootAuto;
import frc.robot.subsystems.intake.FullIntake;
import frc.robot.subsystems.intake.IntakeUntilBreakbeam;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.shooter.Shoot;
import frc.robot.subsystems.swerve.SwerveGoTo;
import frc.robot.subsystems.swerve.ZeroGyro;

/**
 * <p> The robot must be placed so that when the intake drops, it falls directly on the center note on the alliance side.
 * <p>
 * <p> This does 6 things:
 * <p> 1) It primes the intake by getting it down to the ground.
 * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
 * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
 * <p> 4) It runs the bottom and top intake to get the note to the actual shooter, changing the shooter's angle in the process.
 * <p> 5) It gets the shooter back to the correct angle, and then shoots.
 * <p> 6) It moves back 1 meter.
 */
public class TwoNoteAuto extends SequentialCommandGroup {

    /**
     * <p> The robot must be placed so that when the intake drops, it falls directly on top of the center note on the alliance side.
     * <p>
     * <p> This does 6 things:
     * <p> 1) It primes the intake by getting it down to the ground.
     * <p> 2) It gets the shooter to the shooting angle (Constants.SHOOTER_SHOOT_ANGLE rotation)
     * <p> 3) It revs the shooter up and then runs the top intake (shooter intake) to actually fire.
     * <p> 4) It runs the bottom and top intake to get the note to the actual shooter, changing the shooter's angle in the process.
     * <p> 5) It gets the shooter back to the correct angle, and then shoots.
     * <p> 6) It moves back 1 meter.
     */
    public TwoNoteAuto() {
        super(
            // zero, drop intake and shoot first note
            new ZeroGyro(),
            new ShootAuto(), 

            // get shooter to intake angle, and go to 2nd note and intake until reached destination or intake breakbeamed
            new ParallelCommandGroup(
                new GetShooterToAngle(Constants.SHOOTER_INTAKE_ANGLE),

                new ParallelRaceGroup( // doesnt need to drive if intaked
                //meters, note is 114 inches back from front, speaker is ~36 inches, robot ~30, and intake extends out a little more
                    new SwerveGoTo(Robot.getMap().swerve, new Translation2d(-1.4, 0)), // should need to go back ~50 inches or ~1.27m
                    new RunIntake(0.7, 2500)
                    //new IntakeUntilBreakbeam(4000) 
                )
            ),
            
            // intake to pizza and go to shoot position
            new ParallelCommandGroup(
                new FullIntake(Robot.getShuffleboard().getSettingNum("Intake Speed"), Robot.getShuffleboard().getSettingNum("Shooter Intake Speed")),
                new SwerveGoTo(Robot.getMap().swerve, new Translation2d(0, 0)) //go back to start position
                
            ),
            
            // get to shooter angle while revving
            new ParallelRaceGroup(
                new GetShooterToAngle(Constants.SHOOTER_SHOOT_ANGLE),      // Get the shooter to shooting position
                new Shoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed")) // rev motor while shooter is angling
            ),

            // shoot second note and go back
            new IntakeAndShoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed"), 0.5) // change rev to go parallel with shooterangle
        );
    }
}
