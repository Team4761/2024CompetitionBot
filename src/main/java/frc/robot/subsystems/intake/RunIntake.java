package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * <p> This just runs the intake for a set amount of time at a set power
 */
public class RunIntake extends Command {
    

    private double speed;   // The speed to run the intake at as a value between -1 and 1 where + is intake and - is outtake
    private long endTime;   // The time at which the command should end

    private IntakeSubsystem intake; // A pointer to the main Robot's intake.

    private long duration = 2000; // The default time which the command should end at in milliseconds


    /**
     * <p> Runs the intake at with speed {speed} for 2 seconds.
     * @param speed The speed to run the intake with. +1 is 100% power intake. -1 is 100% power outtake.
     */
    public RunIntake(double speed) {
        this.intake = Robot.getMap().intake;
        this.speed = speed;
    }

    /**
     * <p> Runs the intake at with speed {speed} for {duration} milliseconds.
     * @param speed The speed to run the intake with. +1 is 100% power intake. -1 is 100% power outtake.
     * @param duration The duration the command should run for in milliseconds.
     */
    public RunIntake(double speed, long duration) {
        this.intake = Robot.getMap().intake;
        this.speed = speed;
        this.duration = duration;
    }

    public void initialize() {
        this.endTime = System.currentTimeMillis() + duration;
    }


    /**
     * <p> Make sure the intake will actually run.
     */
    @Override
    public void execute() {
        intake.intake(speed);
        System.out.print("1.");
    }

    /**
     * <p> Ends the command after the allotted time has passed
     * @return True if the time has passed. False if the time has NOT passed.
     */
    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() >= endTime) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * <p> Stop the intake once the command ends.
     */
    public void end(boolean isInterrupted) {
        intake.stop();
    }
}
