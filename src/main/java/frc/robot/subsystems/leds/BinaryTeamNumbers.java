package frc.robot.subsystems.leds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class BinaryTeamNumbers extends SequentialCommandGroup {

    public  BinaryTeamNumbers() 
    {
        super(
            showNumber(4),
            Commands.waitSeconds(1),
            showNumber(7),
            Commands.waitSeconds(1),
            showNumber(6),
            Commands.waitSeconds(1),
            showNumber(1)
        );
    }

    private static Command showNumber(int number) {
        return Commands.runOnce(() -> Robot.getMap().leds.binaryLed(number));
    }
}
