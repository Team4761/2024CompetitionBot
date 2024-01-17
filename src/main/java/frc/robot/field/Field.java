package frc.robot.field;

import static frc.robot.field.Constants.*;

public class Field {

    //-------
    // Amp
    //-------
    public static final Amp AMP = new Amp(AMP_TOP_LEFT);

    //-------
    // Speaker
    //-------
    public static final Speaker SPEAKER = new Speaker(SPEAKER_TOP_LEFT, SPEAKER_BOTTOM_RIGHT);
    public static final Subwoofer SUBWOOFER = new Subwoofer(SUBWOOFER_TOP_LEFT, SUBWOOFER_BOTTOM_RIGHT);

    //-------
    // Notes
    //-------

    // Closer to the Speaker
    public static Note NOTE_NEAR_SPEAKER_TOP = new Note(NOTE_NEAR_SPEAKER_TOP_CENTER);
    public static Note NOTE_NEAR_SPEAKER_MIDDLE = new Note(NOTE_NEAR_SPEAKER_MIDDLE_CENTER);
    public static Note NOTE_NEAR_SPEAKER_BOTTOM = new Note(NOTE_NEAR_SPEAKER_BOTTOM_CENTER);

    //Center Notes
    public static Note NOTE_CENTER_TOP = new Note(NOTE_CENTER_TOP_CENTER);
    public static Note NOTE_CENTER_TOP_MIDDLE = new Note(NOTE_CENTER_TOP_MIDDLE_CENTER);
    public static Note NOTE_CENTER_MIDDLE = new Note(NOTE_CENTER_MIDDLE_CENTER);
    public static Note NOTE_CENTER_BOTTOM_MIDDLE = new Note(NOTE_CENTER_BOTTOM_MIDDLE_CENTER);
    public static Note NOTE_CENTER_BOTTOM = new Note(NOTE_CENTER_BOTTOM_CENTER);
}
