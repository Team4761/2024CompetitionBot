package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedSubsystem extends SubsystemBase implements LedSubsystemInterface {
    public static final int LED_WIDTH=8;
    public static final int LED_LENGTH=32;
    public static final int LED_SIZE = LED_WIDTH*LED_LENGTH;

    // --------------------------------------------------
    // Factory pattern
    // --------------------------------------------------
    public static LedSubsystemInterface create() {
        try {
            return new LedSubsystem();
        } catch (Throwable t) {
            t.printStackTrace();
            return new LedSubsystemMock();
        }
    }

    // --------------------------------------------------

    AddressableLED leds;
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_SIZE);

    private LedSubsystem() {
        try {
            leds = new AddressableLED((int)SmartDashboard.getNumber("LED Port", 0));
            leds.setLength(LED_SIZE);
            leds.start();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void SetAllColor(int r, int g, int b) {
        for (int i = 0; i < LED_SIZE; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        leds.setData(ledBuffer);
    }

    @Override
    public void SetRowColor(int row, int r, int g, int b) {
        int start = row * LED_WIDTH;
        for (int i = 0; i < LED_WIDTH; ++i) {
            ledBuffer.setRGB(start + i, r, g, b);
        }
        leds.setData(ledBuffer);
    }

    //Indicator of having a Note and the LEDs beung that indidcator
    @Override
    public void NoteIndicator(boolean HaveNote) {
        if (HaveNote) {
            SetAllColor(250, 90, 0);
        } else {
            SetAllColor(0,0,0);
        }
    }

    @Override
    public void NoteIndicatorOn() {
        NoteIndicator(true);
    }

    @Override
    public void NoteIndicatorOff() {
        NoteIndicator(false);
    }
}


