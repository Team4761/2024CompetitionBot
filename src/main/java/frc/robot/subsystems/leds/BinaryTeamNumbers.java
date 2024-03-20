package frc.robot.subsystems.leds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;

public class BinaryTeamNumbers extends Command {
    

    public  BinaryTeamNumbers() 
    {
        if (Robot.getMap().leds!=null)
        {
            if (Robot.getMap().leds.RGB!= null)
            {
                int zero [] =  new int [Robot.getMap().leds.RGB.length];
                for (int i=0; i<Robot.getMap().leds.RGB.length;i++)
                {
                    zero[i]=0;
                }
                Robot.getMap().leds.SetRowColor(0, 10, 250, 10);
            }  
        }
    }

        
    public void execute() 
    {
        if (Robot.getMap().leds!=null)
        {
            if (Robot.getMap().leds.RGB != null)
            {
                Robot.getMap().leds.four();
                Robot.getMap().leds.seven();
                Robot.getMap().leds.six();
                Robot.getMap().leds.one();
            }  
        }
        else
        {
            isFinished();
        }
    }
        

        
    public boolean isFinished() 
    {
        return true;
    }
}
