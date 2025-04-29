package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.Map;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Vision extends SubsystemBase {
    private final Map<PhotonCamera, PhotonPoseEstimator> poseEstimators;

    private final AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    CommandSwerveDrivetrain drive;

    public Vision(CommandSwerveDrivetrain drive) {
        this.drive = drive;

        poseEstimators =
                CAMERAS.stream()
                        .collect(
                                Collectors.toMap(
                                        camera -> new PhotonCamera(camera.name()),
                                        camera ->
                                                new PhotonPoseEstimator(
                                                        aprilTagFieldLayout,
                                                        PhotonPoseEstimator.PoseStrategy
                                                                .MULTI_TAG_PNP_ON_COPROCESSOR,
                                                        camera.pose())));
    }

    @Override
    public void periodic() {
        poseEstimators.forEach(
                (PhotonCamera camera, PhotonPoseEstimator poseEstimator) ->
                        camera.getAllUnreadResults()
                                .forEach(
                                        (result) ->
                                                poseEstimator
                                                        .update(result)
                                                        .ifPresent(
                                                                (estimatedRobotPose) -> {
                                                                    Logger.recordOutput(
                                                                            "vision/"
                                                                                    + camera
                                                                                            .getName()
                                                                                    + "/pose",
                                                                            estimatedRobotPose
                                                                                    .estimatedPose);
                                                                    Logger.recordOutput(
                                                                            "vision/"
                                                                                    + camera
                                                                                            .getName()
                                                                                    + "/pose2d",
                                                                            estimatedRobotPose
                                                                                    .estimatedPose
                                                                                    .toPose2d());
                                                                    Logger.recordOutput(
                                                                            "vision/"
                                                                                    + camera
                                                                                            .getName()
                                                                                    + "/timestamp",
                                                                            estimatedRobotPose
                                                                                    .timestampSeconds);
                                                                    Logger.recordOutput(
                                                                            "vision/"
                                                                                    + camera
                                                                                            .getName()
                                                                                    + "/latency",
                                                                            Timer.getFPGATimestamp()
                                                                                    - estimatedRobotPose
                                                                                            .timestampSeconds);
                                                                    Logger.recordOutput(
                                                                            "vision/"
                                                                                    + camera
                                                                                            .getName()
                                                                                    + "/strategy",
                                                                            estimatedRobotPose
                                                                                    .strategy);
                                                                    Matrix<N3, N1>
                                                                            visionMeasurementStdDevs;

                                                                    double linearStd;

                                                                    if (estimatedRobotPose.strategy
                                                                            == PhotonPoseEstimator
                                                                                    .PoseStrategy
                                                                                    .MULTI_TAG_PNP_ON_COPROCESSOR) {
                                                                        linearStd = .5;
                                                                    } else {
                                                                        double targetDist =
                                                                                estimatedRobotPose
                                                                                        .targetsUsed
                                                                                        .get(0)
                                                                                        .getBestCameraToTarget()
                                                                                        .getTranslation()
                                                                                        .getNorm();
                                                                        Logger.recordOutput(
                                                                                "vision/"
                                                                                        + camera
                                                                                                .getName()
                                                                                        + "/targetDist",
                                                                                targetDist);
                                                                        linearStd =
                                                                                Math.max(
                                                                                        1.1
                                                                                                        * targetDist
                                                                                                + .125,
                                                                                        .25);
                                                                    }

                                                                    Logger.recordOutput(
                                                                            "vision/"
                                                                                    + camera
                                                                                            .getName()
                                                                                    + "/linearSTD",
                                                                            linearStd);

                                                                    visionMeasurementStdDevs =
                                                                            VecBuilder.fill(
                                                                                    linearStd,
                                                                                    linearStd,
                                                                                    9999999);

                                                                    drive.addVisionMeasurement(
                                                                            estimatedRobotPose
                                                                                    .estimatedPose
                                                                                    .toPose2d(),
                                                                            estimatedRobotPose
                                                                                    .timestampSeconds,
                                                                            visionMeasurementStdDevs);
                                                                })));
    }
}
