package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.util.ScoringPositions;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PositionConstants {
    public static final class Reef {
        public static final Translation2d REEF_CENTER =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(317. / 2.));

        public static final Map<ScoringPositions.Branch, Supplier<Pose2d>> SCORING_POSITIONS =
                new HashMap<>();

        static {
            double adjustX = Units.inchesToMeters(32.745545 + 22 - 3);
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
                ScoringPositions.Branch rightBranch =
                        ScoringPositions.Branch.values()[face * 2 + 1];

                // Add to the map
                SCORING_POSITIONS.put(leftBranch, () -> AllianceFlipUtil.apply(rotatedLeft));
                SCORING_POSITIONS.put(rightBranch, () -> AllianceFlipUtil.apply(rotatedRight));
            }
        }
    }

    public static ScoringPositions.Branch[] getClosestFace(Pose2d robotPose) {
        var angle =
                new Transform2d(
                                new Pose2d(
                                        AllianceFlipUtil.apply(Reef.REEF_CENTER), Rotation2d.kZero),
                                robotPose)
                        .getTranslation()
                        .getAngle()
                        .getDegrees();

        Logger.recordOutput("drive/autoAlign/reefAngle", angle);

        double adjustedAngle;

        if (!AllianceFlipUtil.shouldFlip()) { // Blue side
            var shiftedAngle = angle - 150;

            adjustedAngle = MathUtil.inputModulus(shiftedAngle, 0, 360);
        } else { // Red alliance
            var shiftedAngle = angle + 30;

            adjustedAngle = MathUtil.inputModulus(shiftedAngle, 0, 360);
        }

        var possiblePositions = ScoringPositions.Branch.values();

        Logger.recordOutput("drive/autoAlign/adjustedAngle", adjustedAngle);

        int face = (int) (adjustedAngle / 60);

        Logger.recordOutput("drive/autoAlign/selectedFace", face);

        return new ScoringPositions.Branch[] {
            possiblePositions[face * 2], possiblePositions[face * 2 + 1]
        };

        //
        //        angle = MathUtil.inputModulus(angle, -180, 180);
        //
        //        angle += 30;
        ////
        ////        while (angle >= 60) {
        ////
        ////        }
        //
        //        if (angle > -30 && angle < 30) {
        //            return new ScoringPositions.Branch[] {ScoringPositions.Branch.A,
        // ScoringPositions.Branch.B};
        //        } else if (angle > 30 && angle < ) {}

        //        return new ScoringPositions.Branch[] {ScoringPositions.Branch.A,
        // ScoringPositions.Branch.B};

    }
}
