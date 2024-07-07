package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class MoveIntakeDown extends Command {
    
    private IntakeSubsystem intake;
    private final double SPEED = -0.1;
    
    private long endTime;

    
    public MoveIntakeDown() {
        intake = Robot.getMap().intake;
    }

    @Override
    public void initialize() {
        endTime = System.currentTimeMillis() + 4000;
    }

    @Override
    public void execute() {
        intake.setAngleMotorSpeed(SPEED);
    }

    @Override
    public boolean isFinished() {
        if (endTime <= System.currentTimeMillis())
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setAngleMotorSpeed(0);
    }
}
