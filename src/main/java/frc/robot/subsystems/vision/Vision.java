package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import static frc.robot.Constants.Vision.*;

public class Vision extends SubsystemBase {
    private final Map<PhotonCamera, PhotonPoseEstimator> poseEstimators;

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    Drive drive;

    public Vision(Drive drive) {
        this.drive = drive;

        poseEstimators = CAMERAS.stream().collect(Collectors.toMap(
                camera -> new PhotonCamera(camera.name()),
                camera -> new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera.pose())
        ));


    }

    @Override
    public void periodic() {
        poseEstimators.forEach(
                (PhotonCamera camera, PhotonPoseEstimator poseEstimator) ->
                    camera.getAllUnreadResults().forEach(
                            (result) ->
                            poseEstimator.update(result).ifPresent(
                                    (estimatedRobotPose) -> {
                                        Logger.recordOutput("vision/" + camera.getName() + "/pose", estimatedRobotPose.estimatedPose.toPose2d());
                                        Logger.recordOutput("vision/" + camera.getName() + "/timestamp", estimatedRobotPose.timestampSeconds);
                                        Logger.recordOutput("vision/" + camera.getName() + "/latency", Timer.getFPGATimestamp() - estimatedRobotPose.timestampSeconds);
                                        Logger.recordOutput("vision/" + camera.getName() + "/strategy", estimatedRobotPose.strategy);

                                        drive.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                                    }
                            )
                    )
        );



    }

}

        
    
    
