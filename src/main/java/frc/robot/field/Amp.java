package frc.robot.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static frc.robot.field.Constants.AMP_HOLE_HEIGHT_BOTTOM;
import static frc.robot.field.Constants.AMP_HOLE_HEIGHT_TOP;

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

    public FieldArea3d getHole3d() {
        Translation2d bottomLeft = hole.getBottomLeft();
        Translation2d topRight = hole.topRight();
        return new FieldArea3d(
            new Translation3d(bottomLeft.getX(), bottomLeft.getY(), AMP_HOLE_HEIGHT_BOTTOM),
            new Translation3d(topRight.getX(), topRight.getY(), AMP_HOLE_HEIGHT_TOP)
        );
    }

}
