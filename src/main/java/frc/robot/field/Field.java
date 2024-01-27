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
    // Stage
    //-------
    public static final Stage STAGE =
        new Stage(
            LEFT_STAGE_POST_TOP_LEFT, TOP_STAGE_POST_CENTER, BOTTOM_STAGE_POST_CENTER,
            RIGHT_TRAP_POINT1, RIGHT_TRAP_POINT2,
            TOP_TRAP_POINT1, TOP_TRAP_POINT2,
            BOTTOM_TRAP_POINT1, BOTTOM_TRAP_POINT2
        );

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
