package frc.robot.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static frc.robot.field.Constants.SPEAKER_IN_HEIGHT_BOTTOM;
import static frc.robot.field.Constants.SPEAKER_IN_HEIGHT_TOP;

public class Speaker extends FieldArea {
    public Speaker(Translation2d speakerTopLeft, Translation2d speakerBottomRight) {
        super(speakerTopLeft, speakerBottomRight);
    }

    /**
     * Rough place where we want to shoot the note
     * @return center of the speaker
     */
    public Translation3d getHoleTarget() {
        Translation2d center = getCenter();
        return new Translation3d(
            center.getX(),
            center.getY(),
            SPEAKER_IN_HEIGHT_BOTTOM + (SPEAKER_IN_HEIGHT_TOP - SPEAKER_IN_HEIGHT_BOTTOM) / 2
        );
    }
}
