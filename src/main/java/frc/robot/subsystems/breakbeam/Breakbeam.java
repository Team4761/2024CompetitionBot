package frc.robot.subsystems.breakbeam;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Breakbeam extends SubsystemBase {
    private DigitalInput beam;
    private boolean lastInput = true;
    private boolean inputFlipped = false; // if this input is different from the last

    public Breakbeam(int ID) {
        beam = new DigitalInput(ID);
    }
    
    @Override
    public void periodic() {
        inputFlipped = lastInput!=beam.get();
        lastInput = beam.get();
    }

    public boolean justBroken() {
        return inputFlipped && isBroken();
    }

    public boolean justUnbroken() {
        return inputFlipped && !isBroken();
    }

    public boolean isBroken() {
        return !beam.get();
    }
}
