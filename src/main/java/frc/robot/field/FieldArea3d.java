package frc.robot.field;

import edu.wpi.first.math.geometry.Translation3d;

public class FieldArea3d {
    private final Translation3d corner1;
    private final Translation3d corner2;

    public FieldArea3d(Translation3d corner1, Translation3d corner2) {
        this.corner1 = corner1;
        this.corner2 = corner2;
    }

    public Translation3d getCorner1() {
        return corner1;
    }

    public Translation3d getCorner2() {
        return corner2;
    }
}
