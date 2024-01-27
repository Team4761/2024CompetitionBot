package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;


public class LedChargeUp extends Command {
    
    private long endTime;
    private int row;
    
    private double rowTime;
    private int numberOfRows;
    

    public LedChargeUp (long time) 
    {
        long timeMilis=time*1000;
        this.endTime = System.currentTimeMillis()+timeMilis;
        row=0;
        numberOfRows=LedSubsystem.LED_LENGTH; 
        rowTime=timeMilis/numberOfRows;
    }

    @Override
    public void initialize() 
    {
        
    }

    @Override
    public void execute() {
        if (row<=numberOfRows) 
        {
            if (System.currentTimeMillis()%((rowTime))==0)
            {
                Robot.getMap().leds.RowOn(250, 120, 0);
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

