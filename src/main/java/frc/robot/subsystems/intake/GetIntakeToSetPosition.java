package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class GetIntakeToSetPosition extends Command {
    private final double MARGIN_OF_ERROR = Units.degreesToRadians(5);   // The range of error that the command can stop when reaching.

    private double angleToGoTo; // In radians
    private IntakeSubsystem intake;

    public GetIntakeToSetPosition(double angleRadians) {
        addRequirements(Robot.getMap().intake);
        intake = Robot.getMap().intake;
        angleToGoTo = angleRadians;
    }

    @Override
    public void initialize() {
       intake.goToRotation(new Rotation2d(angleToGoTo));
    }

    @Override
    public boolean isFinished() {
        System.out.println("Rotation:" + Math.abs(intake.getIntakeAngle().getRadians()));
        if (Math.abs(intake.getIntakeAngle().getRadians() - angleToGoTo) <= MARGIN_OF_ERROR)
            return true;
        return false;
    }
}
