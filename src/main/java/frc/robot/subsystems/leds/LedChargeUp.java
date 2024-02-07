package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;


public class LedChargeUp extends Command {
    
    private long endTime;
    private int row;
    private int i=0;
    private double rowTime;
    private int numberOfRows;
    

    public LedChargeUp () 
    {
       
    }

    @Override
    public void initialize() 
    {
        
    }

    @Override
    public void execute() {
        if (row<=numberOfRows) 
        {
            i++;
            if(i%0.125==0)
            {
                Robot.getMap().leds.RowOn(row,250, 120, 0);
                row++;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (endTime <= System.currentTimeMillis())
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}

