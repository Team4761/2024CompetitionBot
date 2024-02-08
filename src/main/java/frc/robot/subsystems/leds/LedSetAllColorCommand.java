package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;

public class LedSetAllColorCommand extends Command {
    private final LedSubsystem ledSubsystem;
    private final int r;
    private final int b;
    private final int g;

    public LedSetAllColorCommand(LedSubsystem ledSubsystem, int r, int b, int g) {
        setSubsystem(ledSubsystem.getSubsystem());
        this.ledSubsystem = ledSubsystem;
        this.r = r;
        this.b = b;
        this.g = g;
    }

    @Override
    public void execute() {
        ledSubsystem.SetAllColor(r, g, b);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
