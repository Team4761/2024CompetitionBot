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


// may need to 
public class FourNoteAuto extends SequentialCommandGroup {
    public FourNoteAuto() {
        super(
            new ThreeNoteAuto(),

            // go to and intake right (when looking at alliance wall) note
            new ParallelCommandGroup(
                new GetShooterToAngle(Constants.SHOOTER_INTAKE_ANGLE),
                
                new ParallelDeadlineGroup( 
                    new SequentialCommandGroup(
                        new WaitCommand(0.15),
                        new SwerveGoTo(Robot.getMap().swerve, new Translation2d(1.35, -1.5))
                    ),
                    new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(-Math.PI*0.75)),
                    new RunIntake(0.7, 3000)
                    //new IntakeUntilBreakbeam(4000) 
                )
            ),

            // intake and go back to 0,0
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new FullIntake(Constants.AUTO_INTAKE_SPEED, Constants.AUTO_UPTAKE_SPEED, new Rotation2d(Constants.SHOOTER_SHOOT_ANGLE)),
                    new SwerveGoTo(Robot.getMap().swerve, new Translation2d(0.2, 0)),
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
            new IntakeAndShoot(40, 0.0) // change rev to go parallel with shooterangle
        );
    }
}
