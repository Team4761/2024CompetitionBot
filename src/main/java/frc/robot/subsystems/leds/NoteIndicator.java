package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// ALL OF THIS IS ROUGH CODE. THE 2000 IS 2 seconds and is TEMPORARY!
public class NoteIndicator extends Command {
    

    public  NoteIndicator() 
    {
    }

        
    public void execute() 
    {
        if (Robot.getMap().shooter.isPieceInUpperIntake()==true)
        {
            Robot.getMap().leds.SetAllColor(250, 90, 0);
            isFinished();
        }  
    }
        

        
    public boolean isFinished() 
    {
        return true;
    }
}
