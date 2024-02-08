package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.subsystems.leds.LedSubsystem.LED_LENGTH;

// ALL OF THIS IS ROUGH CODE. THE 2000 IS 2 seconds and is TEMPORARY!
public class LedChargeUp extends SequentialCommandGroup {
    private static final int BANK_SIZE = 2;

    public LedChargeUp(double time, LedSubsystem subsystem) {
        int numIncrements = LED_LENGTH / BANK_SIZE;
        double timeEachIncrement = time / numIncrements;
        addCommands(new LedSetAllColorCommand(subsystem, 0, 0, 0));
        for (int increment = 0; increment < numIncrements; increment++) {
            addCommands(new LedBankCommand(subsystem, increment).withTimeout(timeEachIncrement));
        }
        addCommands(new LedSetAllColorCommand(subsystem, 0, 0, 0));
    }

    private static class LedBankCommand extends Command {
        private final int increment;
        private final LedSubsystem subsystem;

        public LedBankCommand(LedSubsystem subsystem, int increment) {
            this.setSubsystem(subsystem.getName());
            this.subsystem = subsystem;
            this.increment = increment;
        }

        @Override
        public void execute() {
            for(int i = 0; i< BANK_SIZE; i++) {
                subsystem.SetRowColor((increment*BANK_SIZE) + i,250, 220, 0);
            }
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}

