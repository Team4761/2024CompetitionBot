package frc.robot.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static frc.robot.field.Constants.*;

public class Amp extends FieldArea {
    private final FieldArea hole;
    public Amp(Translation2d startPos) {
        super(startPos, startPos.plus(new Translation2d(Constants.AMP_WIDTH, 0)));
        Translation2d holePos = startPos.plus(new Translation2d(3, 0));
        hole = new FieldArea(holePos, holePos.plus(new Translation2d(Constants.AMP_HOLE_WIDTH, 0)));
    }

    public FieldArea getHole() {
        return hole;
    }

    /**
     * Best guess where to insert the note into the Amp (dead center of the hole)
     */
    public Translation3d getHoleTarget() {
        Translation2d bottomCenter = hole.getBottomCenter();
        return new Translation3d(
            bottomCenter.getX(),
            bottomCenter.getY(),
            AMP_HOLE_HEIGHT_BOTTOM + (AMP_HOLE_HEIGHT / 2)
        );
    }

}
