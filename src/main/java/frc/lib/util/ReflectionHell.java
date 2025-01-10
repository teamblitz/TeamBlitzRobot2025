package frc.lib.util;

public final class ReflectionHell {
    //    private static Field poseBufferField;
    //    private static Field pose2dField;
    //
    //    static {
    //        try {
    //            poseBufferField = PoseEstimator.class.getDeclaredField("m_poseBuffer");
    //            poseBufferField.setAccessible(true);
    //
    //            pose2dField =
    //
    // PoseEstimator.class.getDeclaredClasses()[0].getDeclaredField("poseMeters");
    //            pose2dField.setAccessible(true);
    //
    //        } catch (NoSuchFieldException | SecurityException | IllegalArgumentException e) {
    //            e.printStackTrace();
    //        }
    //    }

    //    private static Optional<Pose2d> samplePoseEstimator(
    //            PoseEstimator<?> poseEstimator, double time) {
    //        try {
    //            var poseBuffer = (TimeInterpolatableBuffer<?>) poseBufferField.get(poseEstimator);
    //            var sample = poseBuffer.getSample(time);
    //
    //            if (sample.isEmpty()) return Optional.empty();
    //
    //            return Optional.of((Pose2d) pose2dField.get(sample.get()));
    //        } catch (IllegalAccessException e) {
    //            e.printStackTrace();
    //            return Optional.empty();
    //        }
    //    }

    private ReflectionHell() {}
}
