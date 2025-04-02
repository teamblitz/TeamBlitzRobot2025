package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.util.ScoringPositions;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PositionConstants {
    public static final class Reef {
        public static final Translation2d REEF_CENTER =
                new Translation2d(
                        Units.inchesToMeters(176.746),
                        Units.inchesToMeters(317. / 2.)
                );

        public static final Map<ScoringPositions.Branch, Supplier<Pose2d>> SCORING_POSITIONS = new HashMap<>();

        static {
            double adjustX = Units.inchesToMeters(32.745545 + 22);
            double adjustY = Units.inchesToMeters(6.469);

            Translation2d leftRelative = new Translation2d(-adjustX, adjustY);
            Translation2d rightRelative = new Translation2d(-adjustX, -adjustY);

            for (int face = 0; face < 6; face++) {
                Rotation2d rotation = Rotation2d.fromDegrees(60 * face);

                // Rotate the relative positions
                Translation2d rotatedLeftRelative = leftRelative.rotateBy(rotation);
                Translation2d rotatedRightRelative = rightRelative.rotateBy(rotation);

                // Convert to absolute positions by adding the reef center
                Translation2d leftAbsolute = REEF_CENTER.plus(rotatedLeftRelative);
                Translation2d rightAbsolute = REEF_CENTER.plus(rotatedRightRelative);

                // Create the poses
                Pose2d rotatedLeft = new Pose2d(leftAbsolute, rotation);
                Pose2d rotatedRight = new Pose2d(rightAbsolute, rotation);

                // Determine the corresponding branches
                ScoringPositions.Branch leftBranch = ScoringPositions.Branch.values()[face * 2];
                ScoringPositions.Branch rightBranch = ScoringPositions.Branch.values()[face * 2 + 1];

                // Add to the map
                SCORING_POSITIONS.put(leftBranch, () -> AllianceFlipUtil.apply(rotatedLeft));
                SCORING_POSITIONS.put(rightBranch, () -> AllianceFlipUtil.apply(rotatedRight));
            }
        }
    }
}
