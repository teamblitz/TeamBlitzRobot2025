package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.*;

import org.photonvision.EstimatedRobotPose;

public final class VisionUtils {

    public static Matrix<N3, N1> calculateStdDevs(
            EstimatedRobotPose est,
            Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevs,
            double maxSingleTagDist,
            double maxMultiTagDist,
            double distanceScaleFactor) {
        var estStdDevs = singleTagStdDevs;
        var targets = est.targetsUsed;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            numTags++;
            avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
        }

        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = multiTagStdDevs;
        // Increase std devs based on (average) distance
        if ((numTags == 1 && avgDist > maxSingleTagDist)
                || (numTags > 1 && avgDist > maxMultiTagDist)) // 4
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / distanceScaleFactor)); // 30

        return estStdDevs;
    }

    private VisionUtils() {}
}
