package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.*;

import org.photonvision.EstimatedRobotPose;

public final class VisionUtils {

    public static Matrix<N3, N1> calculateStdDevs(
            EstimatedRobotPose est) {

        var targets = est.targetsUsed;
        int numTags = targets.size();

        double avgDist = 0;
        for (var target : targets) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= numTags;

        if (numTags == 0) return VisionConstants.NO_TRUST;

        if (numTags == 1 && avgDist > VisionConstants.MAX_DISTANCE_SINGLE_TAG
                || numTags > 1 && avgDist > VisionConstants.MAX_DISTANCE_MULTI_TAG)
            return VisionConstants.NO_TRUST;

        var estStdDevs = numTags > 1 ? VisionConstants.MULTI_TAG_TRUST : VisionConstants.SINGLE_TAG_TRUST;


        return estStdDevs.times(
                1 + Math.pow(avgDist, VisionConstants.ORDER) * VisionConstants.PROPORTION
        );
    }


    private VisionUtils() {}
}
