package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveBackCommand extends SequentialCommandGroup{
    
    public MoveBackCommand(Command moveBackCommand){
        super(moveBackCommand);
    }
}
