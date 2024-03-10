package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeAndLaunchPiece extends Command {
    
    private double intakeBottomSpeed;       // The speed the intake should run at as a value between -1 and 1. Positive is intake.
    private double intakeTopSpeed;          // The speed that the shooter's intake should run at as a value between -1 and 1. Positive is intake.
    private double shootSpeed;              // The speed which the shooter should run at in rotations per second.

    private IntakeSubsystem intake;     // A reference to the intake subsystem
    private ShooterSubsystem shooter;   // A reference to the shooter subsystem

    private boolean hadPieceInShooter;  // Keeps track of if there EVER was a piece in the top of the shooter.
    private long endTime;


    /**
     * <p> This auto runs the full intake system (both intake and shooter) while running the shooter as well.
     * <p> This command ends 1 second after the top break beam of the shooter is broken.
     * @param intakeSpeed The speed to run both the shooter's and intake's intake speed as a value between -1 and 1. Positive means intake, negative is outtake.
     * @param shooterSpeed The speed to run the shooter at in rotations per second. Max is around 50 RPS.
     */
    public IntakeAndLaunchPiece(double intakeSpeed, double shootSpeed) {
        this.intakeBottomSpeed = intakeSpeed;
        this.intakeTopSpeed = intakeSpeed;
        this.shootSpeed = shootSpeed;

        this.hadPieceInShooter = false; // Starts at false, becomes true once the top break beam is broken.
        this.endTime = 0;   // Placeholder value; does not mean anything, but it does fix potential errors of not being initialized.
    }

    
    @Override
    public void execute() {
        intake.intake(intakeBottomSpeed);
        shooter.setIntakeSpeed(intakeTopSpeed);
        shooter.setShooterSpeed(shootSpeed);
        
        // If the break beam is broken, start the end of the command timer
        if (!hadPieceInShooter && shooter.isPieceInUpperIntake()) {
            hadPieceInShooter = true;
            endTime = System.currentTimeMillis() + 1000;
        }
    }

    @Override
    public boolean isFinished() {
        if (hadPieceInShooter && System.currentTimeMillis() >= endTime) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        intake.stop();
        shooter.setIntakeSpeed(0);
        shooter.setShooterSpeed(0);
    }
}
