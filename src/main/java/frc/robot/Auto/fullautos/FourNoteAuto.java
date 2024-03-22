package frc.robot.Auto.fullautos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.FullIntake;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.IntakeAndShoot;
import frc.robot.subsystems.shooter.Shoot;
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
                    new SwerveGoTo(Robot.getMap().swerve, new Translation2d(-1.4, -1.5)), 
                    new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(-Math.PI/4)),
                    new RunIntake(0.7, 3000)
                    //new IntakeUntilBreakbeam(4000) 
                )
            ),

            // intake and go back to 0,0
            new ParallelCommandGroup(
                new FullIntake(Robot.getShuffleboard().getSettingNum("Intake Speed"), Robot.getShuffleboard().getSettingNum("Shooter Intake Speed")),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new ParallelCommandGroup(
                        new SwerveGoTo(Robot.getMap().swerve, new Translation2d(0, 0)),
                        new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d())
                    )
                )
            ),

            // get shooter to angle and rev
            new ParallelRaceGroup(
                new GetShooterToAngle(Constants.SHOOTER_SHOOT_ANGLE),      // Get the shooter to shooting position
                new Shoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed")) // rev motor while shooter is angling
            ),

            // shoot
            new IntakeAndShoot(Robot.getShuffleboard().getSettingNum("Shooter Out Speed"), 0.5) // change rev to go parallel with shooterangle
        );
    }
}
