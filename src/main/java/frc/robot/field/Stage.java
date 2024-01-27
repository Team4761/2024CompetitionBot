package frc.robot.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static frc.robot.field.Constants.*;

public class Stage {
    private final FieldArea sideStagePost;
    private final FieldArea topStagePost;
    private final FieldArea bottomStagePost;
    private final FieldArea sideTrap;
    private final FieldArea topTrap;
    private final FieldArea bottomTrap;

    public Stage(
        Translation2d sideStageTopLeftPos,
        Translation2d topStagePostCenter,
        Translation2d bottomStagePostCenter,
        Translation2d sideTrapStart, Translation2d sideTrapEnd,
        Translation2d topTrapStart, Translation2d topTrapEnd,
        Translation2d bottomTrapStart, Translation2d bottomTrapEnd
    ) {
        Translation2d postWidth = new Translation2d(POST_SIZE, POST_SIZE);
        Translation2d postHalfWidth = new Translation2d(POST_SIZE / 2, POST_SIZE / 2);

        sideStagePost = new FieldArea(sideStageTopLeftPos, sideStageTopLeftPos.plus(postWidth));
        topStagePost =
            new FieldArea(topStagePostCenter.minus(postHalfWidth), topStagePostCenter.plus(postHalfWidth));
        bottomStagePost =
            new FieldArea(bottomStagePostCenter.minus(postHalfWidth), bottomStagePostCenter.plus(postHalfWidth));

        this.sideTrap = new FieldArea(sideTrapStart, sideTrapEnd);
        this.topTrap = new FieldArea(topTrapStart, topTrapEnd);
        this.bottomTrap = new FieldArea(bottomTrapStart, bottomTrapEnd);

        // TODO: chains
    }

    public FieldArea getSideStagePost() {
        return sideStagePost;
    }

    public FieldArea getTopStagePost() {
        return topStagePost;
    }

    public FieldArea getBottomStagePost() {
        return bottomStagePost;
    }

    public Translation3d getSideTrapTarget() {
        return getTrapTarget(sideTrap);
    }

    public Translation3d getTopTrapTarget() {
        return getTrapTarget(topTrap);
    }

    public Translation3d getBottomTrapTarget() {
        return getTrapTarget(bottomTrap);
    }

    private Translation3d getTrapTarget(FieldArea sideTrap) {
        Translation2d center = sideTrap.getCenter();
        return new Translation3d(center.getX(), center.getY(), TRAP_TARGET_HEIGHT);
    }
}
