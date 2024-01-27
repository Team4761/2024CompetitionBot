package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

// ALL OF THIS IS ROUGH CODE. THE 2000 IS 2 seconds and is TEMPORARY!
public class LedChargeUp extends Command {
    private double speed; // Value between -1 and 1
    private long endTime;
    private int row;
    private int rowLength;
    private double rowTime;
    private int numberOfRows;

    public LedChargeUp (long time) {
        row=0;
        this.endTime = System.currentTimeMillis()+time;
        rowLength=LedSubsystem.LED_WIDTH;
        numberOfRows=LedSubsystem.LED_LENGTH; 
        rowTime=time/numberOfRows;

        // time for whole thing = time / # of rows
        // time for each row = time /# of leds in a row
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Robot.getMap().shooter.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        if (endTime <= System.currentTimeMillis())
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.getMap().shooter.setSpeed(0);
    }
}

