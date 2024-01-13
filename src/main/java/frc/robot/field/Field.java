package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Field {
    //Orgin 
    public static double Origin=0.0;

    //Filler
    public static double Filler=0.0;

    //Field length, width, and area
    public static double FL=(54*12 + 3.75);
    public static double FW=(26*12 + 11.25);
    public static double FA=(FL*FW);

    //Center Pos
    public static double CenterX=FL/2;
    public static double CenterY=FW/2;

    //Source (where we get notes) Pos.  Source is abveraited to Src
    public static Translation2d SRC_TOP_LEFT = new Translation2d(0, 197.5);
    public static Translation2d SRC_BOTTOM_RIGHT = new Translation2d(18, 238.5);

    //Source shelf
    public static Translation2d SRC_SHELF_TOP_LEFT = new Translation2d(0, 200);
    public static Translation2d SRC_SHELF_BOTTOM_RIGHT = new Translation2d(36, 236);

    //Amp
    public static Translation2d AMP_TOP_LEFT = new Translation2d(Filler,Filler);
    public static Translation2d AMP_BOTTEM_RIGHT = new Translation2d(Filler,Filler);
  
    //Amp Slot
    public static Translation2d AMP_HOLE_TOP_LEFT = new Translation2d(60.5, 323);
    public static Translation2d AMP_HOLE_BOTTOM_RIGHT = new Translation2d(84.5, 323);
    public static double AMP_HOLE_HEIGHT_BOTTOM = 26;
    public static double AMP_HOLE_HEIGHT_TOP = 44;

    //Speaker
    public static Translation2d SPEAKER_TOP_LEFT = new Translation2d(0, 197.5);
    public static Translation2d SPEAKER_BOTTOM_RIGHT = new Translation2d(18, 238.5);
    public static Translation2d SUBWOOFER_TOP_LEFT = new Translation2d(0, 200);
    public static Translation2d SUBWOOFER_BOTTOM_RIGHT = new Translation2d(36, 236);
    
    public static double SPEAKER_IN_HEIGHT_BOTTOM = 6*12 + 6;
    public static double SPEAKER_IN_HEIGHT_TOP = 6*12 + 11;

    //Stage

    //Top Most Stage Post
    public static Translation2d TOP_STAGE_POST_TOP_LEFT = new Translation2d(Filler,Filler);
    public static Translation2d TOP_STAGE_POST_BOTTOM_LEFT = new Translation2d(Filler,Filler);

    //Bottom Most Stage Post
    public static Translation2d BOTTOM_STAGE_POST_TOP_LEFT = new Translation2d(Filler,Filler);
    public static Translation2d BOTTOM_STAGE_POST_BOTTOM_RIGHT = new Translation2d(Filler,Filler);

    //Left Most Stage Post
    public static Translation2d LEFT_STAGE_POST_TOP_LEFT = new Translation2d(Filler,Filler);
    public static Translation2d LEFT_STAGE_POST_BOTTOM_RIGHT = new Translation2d(Filler,Filler);

    //Trap

    //Top Most Trap
    public static Translation2d TOP_TRAP_TOP_LEFT = new Translation2d(Filler,Filler);
    public static Translation2d TOP_TRAP_BOTTOM_RIGHT = new Translation2d(Filler,Filler);

    //Bottom Most Trap
    public static Translation2d BOTTOM_TRAP_TOP_LEFT = new Translation2d(Filler,Filler);
    public static Translation2d BOTTOM_TRAP_BOTTOM_RIGHT = new Translation2d(Filler,Filler);

    //Right Most Trap
    public static Translation2d RIGHT_TRAP_TOP_LEFT = new Translation2d(Filler,Filler);
    public static Translation2d RIGHT_TRAP_BOTTOM_RIGHT = new Translation2d(Filler,Filler);

    //Note Postions

    //Near Speaker Notes (Top is from horrizontal view) 
    public static Translation2d NOTE_NEAR_SPEAKER_TOP = new Translation2d(114,276);
    public static Translation2d NOTE_NEAR_SPEAKER_MIDDLE = new Translation2d(114, 219);
    public static Translation2d NOTE_NEAR_SPEAKER_BOTTOM = new Translation2d(114,162);
    //Center Notes
    public static Translation2d NOTE_CENTER_TOP = new Translation2d(324,294);
    public static Translation2d NOTE_CENTER_TOP_MIDDLE = new Translation2d(324,228);
    public static Translation2d NOTE_CENTER_MIDDLE = new Translation2d(324,162);
    public static Translation2d NOTE_CENTER_BOTTOM_MIDDLE = new Translation2d(324,96);
    public static Translation2d NOTE_CENTER_BOTTOM = new Translation2d(324,30);

    
}
