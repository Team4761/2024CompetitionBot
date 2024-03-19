package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FlowLEDs extends Command {
    
    private long TIME_PER_CYCLE = 50;

    private LedSubsystem leds;
    
    private final Color MAIN = new Color(100,0,0);
    private final Color SECONDARY = new Color(100,40,40);

    private Color[] buffer;

    private double cycle;
    private int maxCycle;
    private int numberOfLEDs;

    private double[] offsets;

    private long nextTime;


    public FlowLEDs() {
        leds = Robot.getMap().leds;

        cycle = 0;
        maxCycle = 100;
        numberOfLEDs = Robot.getMap().leds.getNumberOfLEDs();
        buffer = new Color[numberOfLEDs];
        offsets = new double[]{(double)(MAIN.red*255-SECONDARY.red*255) / maxCycle, (double)(MAIN.green*255-SECONDARY.green*255) / maxCycle, (double)(MAIN.blue*255-SECONDARY.blue*255)};
        nextTime = System.currentTimeMillis() + TIME_PER_CYCLE;
    }


    @Override
    public void execute() {
        if (nextTime <= System.currentTimeMillis()) {
            for (int i = 0; i < numberOfLEDs; i++) {
                buffer[i] = new Color(
                    (MAIN.red) + (offsets[0] * ((cycle+i)%maxCycle)),
                    (MAIN.green) + (offsets[1] * ((cycle+i)%maxCycle)),
                    (MAIN.blue) + (offsets[2] * ((cycle+i)%maxCycle)));
            }
            leds.setLEDs(buffer);

            cycle++;

            if (cycle >= maxCycle) {
                cycle = 0;
            }

            nextTime = System.currentTimeMillis() + TIME_PER_CYCLE;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
