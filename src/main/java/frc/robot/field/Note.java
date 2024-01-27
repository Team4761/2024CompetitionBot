package frc.robot.field;

import edu.wpi.first.math.geometry.Translation2d;

public class Note extends FieldArea {
    public static final double DIAMETER = 14;
    public Note(Translation2d center) {
        super(
            center.minus(new Translation2d(DIAMETER / 2, DIAMETER / 2)),
            center.plus(new Translation2d(DIAMETER / 2, DIAMETER / 2))
        );
    }
}
