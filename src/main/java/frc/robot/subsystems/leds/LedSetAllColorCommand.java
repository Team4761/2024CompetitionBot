package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class LedSetAllColorCommand extends Command {
    private final RobotMap robotMap;
    private final int r;
    private final int b;
    private final int g;

    public LedSetAllColorCommand(RobotMap robotMap, int r, int b, int g) {
        this.robotMap = robotMap;
        this.r = r;
        this.b = b;
        this.g = g;
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
