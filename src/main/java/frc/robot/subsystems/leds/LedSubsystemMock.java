package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystemMock extends SubsystemBase implements LedSubsystemInterface {
    @Override
    public void SetAllColor(int r, int g, int b) {

    }

    @Override
    public void SetRowColor(int row, int r, int g, int b) {

    }

    @Override
    public void NoteIndicator(boolean HaveNote) {

    }

    @Override
    public void NoteIndicatorOn() {
        NoteIndicator(true);
    }

    public void NoteIndicatorOff() {
        NoteIndicator(false);
    }
}
