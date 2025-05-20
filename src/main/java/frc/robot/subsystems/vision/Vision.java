package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class Vision extends SubsystemBase {
    private final Map<PhotonCamera, PhotonPoseEstimator> poseEstimators;

    public static final AprilTagFieldLayout DEFAULT_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final AprilTagFieldLayout DEMO_FOLLOW_LAYOUT = new AprilTagFieldLayout(
            List.of(new AprilTag(
                    0, new Pose3d(new Translation3d(), new Rotation3d(0, 0, Math.PI)))),
            DEFAULT_FIELD_LAYOUT.getFieldLength(),
            DEFAULT_FIELD_LAYOUT.getFieldWidth());

    CommandSwerveDrivetrain drive;

    public Vision(CommandSwerveDrivetrain drive) {
        this.drive = drive;

        drive.setStateStdDevs(VisionConstants.WHEEL_ODOMETRY_TRUST);

        poseEstimators = CAMERAS.stream()
                .collect(Collectors.toMap(
                        camera -> new PhotonCamera(camera.name()),
                        camera -> new PhotonPoseEstimator(
                                DEFAULT_FIELD_LAYOUT,
                                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                camera.pose())));
    }

    @Override
    public void periodic() {
        List<PoseObservation> estimations = new ArrayList<>();

        poseEstimators.forEach((PhotonCamera camera, PhotonPoseEstimator poseEstimator) ->
                camera.getAllUnreadResults().forEach((result) -> poseEstimator
                        .update(result)
                        .ifPresent((estimatedRobotPose) -> {
                            var logKey = "vision/" + camera.getName();

                            Logger.recordOutput(logKey + "/pose", estimatedRobotPose.estimatedPose);
                            Logger.recordOutput(
                                    logKey + "/pose2d",
                                    estimatedRobotPose.estimatedPose.toPose2d());
                            Logger.recordOutput(
                                    logKey + "/timestamp", estimatedRobotPose.timestampSeconds);
                            Logger.recordOutput(
                                    logKey + "/latency",
                                    Timer.getFPGATimestamp() - estimatedRobotPose.timestampSeconds);
                            Logger.recordOutput(logKey + "/strategy", estimatedRobotPose.strategy);

                            var obs = new PoseObservation(
                                    estimatedRobotPose,
                                    VisionUtils.calculateStdDevs(estimatedRobotPose));

                            Logger.recordOutput(logKey + "/stdDev", new double[] {
                                obs.stdDevs.get(0, 0), obs.stdDevs.get(1, 0), obs.stdDevs.get(2, 0)
                            });

                            estimations.add(new PoseObservation(
                                    estimatedRobotPose,
                                    VisionUtils.calculateStdDevs(estimatedRobotPose)));
                        })));

        estimations.sort(Comparator.comparingDouble(observation -> observation.stdDevs.get(0, 0)));

        estimations.forEach(poseObservation -> drive.addVisionMeasurement(
                poseObservation.pose.estimatedPose.toPose2d(),
                poseObservation.pose.timestampSeconds,
                poseObservation.stdDevs));
    }

    public void setAprilTagLayout(AprilTagFieldLayout aprilTagFieldLayout) {
        poseEstimators
                .values()
                .forEach(photonPoseEstimator ->
                        photonPoseEstimator.setFieldTags(aprilTagFieldLayout));
    }

    private record PoseObservation(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {}
}
