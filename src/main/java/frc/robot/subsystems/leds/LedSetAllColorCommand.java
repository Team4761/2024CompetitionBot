package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class LedSetAllColorCommand extends Command {
    private final RobotMap robotMap;
    private final int r;
    private final int g;
    private final int b;
    

    public LedSetAllColorCommand(int r, int g, int b) {
        this.robotMap = Robot.getMap();
        this.r = r;
        this.g = g;
        this.b = b;   
    }

    @Override
    public void execute() {
        robotMap.leds.SetAllColor(r, g, b);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
