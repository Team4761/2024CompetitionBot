package frc.robot.field;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    //Origin
    public static final double Origin=0.0;

    //Filler
    public static final double Filler=0.0;

    //Field length, width, and area
    public static final double FIELD_HEIGHT=(54*12 + 3.75);
    public static final double FIELD_WIDTH =(26*12 + 11.25);

    //Center Pos
    public static final Translation2d CENTER = new Translation2d(FIELD_WIDTH / 2, FIELD_HEIGHT / 2);

    //Source (where we get notes) Pos.  Source is abveraited to Src
    public static final Translation2d SRC_TOP_LEFT = new Translation2d(0, 197.5);
    public static final Translation2d SRC_BOTTOM_RIGHT = new Translation2d(18, 238.5);

    //Source shelf
    public static final Translation2d SRC_SHELF_TOP_LEFT = new Translation2d(0, 200);
    public static final Translation2d SRC_SHELF_BOTTOM_RIGHT = new Translation2d(36, 236);

    //Amp
    public static final Translation2d AMP_TOP_LEFT = new Translation2d(57.5, 323);

    //Amp Slot
    public static final double AMP_HOLE_WIDTH = 14;
    public static final double AMP_WIDTH = AMP_HOLE_WIDTH + 6;
    public static final double AMP_HOLE_HEIGHT_BOTTOM = 26;
    public static final double AMP_HOLE_HEIGHT_TOP = 44;

    //Speaker
    public static final Translation2d SPEAKER_TOP_LEFT = new Translation2d(0, 197.5);
    public static final Translation2d SPEAKER_BOTTOM_RIGHT = new Translation2d(18, 238.5);
    public static final Translation2d SUBWOOFER_TOP_LEFT = new Translation2d(0, 200);
    public static final Translation2d SUBWOOFER_BOTTOM_RIGHT = new Translation2d(36, 236);

    public static final double SPEAKER_IN_HEIGHT_BOTTOM = 6*12 + 6;
    public static final double SPEAKER_IN_HEIGHT_TOP = 6*12 + 11;

    //Stage

    //Top Most Stage Post
    public static final Translation2d LEFT_STAGE_POST_TOP_LEFT = new Translation2d(121, 157);
    public static final Translation2d TOP_STAGE_POST_CENTER = new Translation2d(225, 218);
    public static final Translation2d BOTTOM_STAGE_POST_CENTER = new Translation2d(225, 105);
    public static final double POST_WIDTH = 10;

    //Trap

    //Top Most Trap
    public static Translation2d TOP_TRAP_POINT1 = new Translation2d(171, 165);
    public static Translation2d TOP_TRAP_POINT2 = new Translation2d(195, 189);

    //Bottom Most Trap
    public static Translation2d BOTTOM_TRAP_POINT1 = new Translation2d(171, 158);
    public static Translation2d BOTTOM_TRAP_POINT2 = new Translation2d(195, 134);

    //Right Most Trap
    public static Translation2d RIGHT_TRAP_POINT1 = new Translation2d(209, 150);
    public static Translation2d RIGHT_TRAP_POINT2 = new Translation2d(209, 174);

    //Note Postions

    //Near Speaker Notes (Top is from horrizontal view)
    public static Translation2d NOTE_NEAR_SPEAKER_TOP_CENTER = new Translation2d(114,276);
    public static Translation2d NOTE_NEAR_SPEAKER_MIDDLE_CENTER = new Translation2d(114, 219);
    public static Translation2d NOTE_NEAR_SPEAKER_BOTTOM_CENTER = new Translation2d(114,162);

    //Center Notes
    public static Translation2d NOTE_CENTER_TOP_CENTER = new Translation2d(324,294);
    public static Translation2d NOTE_CENTER_TOP_MIDDLE_CENTER = new Translation2d(324,228);
    public static Translation2d NOTE_CENTER_MIDDLE_CENTER = new Translation2d(324,162);
    public static Translation2d NOTE_CENTER_BOTTOM_MIDDLE_CENTER = new Translation2d(324,96);
    public static Translation2d NOTE_CENTER_BOTTOM_CENTER = new Translation2d(324,30);
}
