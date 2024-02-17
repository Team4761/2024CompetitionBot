package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface LedSubsystemInterface extends Subsystem {
    void SetAllColor(int r, int g, int b);

    void SetRowColor(int row, int r, int g, int b);

    //Indicator of having a Note and the LEDs beung that indidcator
    void NoteIndicator(boolean HaveNote);

    void NoteIndicatorOn();
    void NoteIndicatorOff();
}
