package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shoot;

public class ShootThenMoveCommand extends SequentialCommandGroup{
    public ShootThenMoveCommand(Shoot shoot, Command move){
        super(shoot, move);
    }
}