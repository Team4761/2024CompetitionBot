package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class GetIntakeToSetPosition extends Command {
    private final double MARGIN_OF_ERROR = Units.degreesToRadians(15);   // The range of error that the command can stop when reaching.
    private final long TIMEOUT = 4000;

    private double angleToGoTo; // In radians
    private IntakeSubsystem intake;
    private long timeOut;

    public GetIntakeToSetPosition(double angleRadians) {
        addRequirements(Robot.getMap().intake);
        intake = Robot.getMap().intake;
        angleToGoTo = angleRadians;
    }

    @Override
    public void initialize() {
        timeOut = System.currentTimeMillis() + TIMEOUT;
        intake.setAngleMotorSpeed( -(Math.signum(intake.getIntakeAngle().getRadians() - angleToGoTo)) * 0.4 );
    //    intake.goToRotation(new Rotation2d(angleToGoTo));

    }

    @Override
    public boolean isFinished() {
        // System.out.println("Rotation:" + Math.abs(intake.getIntakeAngle().getRadians()));
        if (Math.abs(intake.getIntakeAngle().getRadians() - angleToGoTo) <= MARGIN_OF_ERROR)
            return true;
        if (System.currentTimeMillis() >= timeOut)
            return true;
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        intake.setAngleMotorSpeed(0);
    }
}
