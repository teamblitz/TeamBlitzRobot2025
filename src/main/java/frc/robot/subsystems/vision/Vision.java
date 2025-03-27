package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

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
    private final List<PhotonCamera> cameras;
    private final Map<PhotonCamera, PhotonPoseEstimator> poseEstimators;

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    Drive drive;

    public Vision(Drive drive) {
        this.drive = drive;

        cameras = List.of(
                new PhotonCamera("OV2311_20.0"),
                new PhotonCamera("OV2311_20.1")
//                new PhotonCamera("2"),
//                new PhotonCamera("3")
        );

        this.poseEstimators = Map.ofEntries(
                Map.entry(cameras.get(0), new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA_POSES.get(0))),
                Map.entry(cameras.get(1), new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA_POSES.get(1)))
//                Map.entry(cameras.get(2), new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA_POSES.get(2))),
//                Map.entry(cameras.get(3), new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA_POSES.get(3)))
        );


    }

//    public Pose2d getAprilTagPose2d() {
//        PhotonTrackedTarget goodTarget = null;
//        PhotonCamera bestCam = null;
//
//        for (PhotonCamera cams : cameras)
//        {
//            var result = cams.getLatestResult();
//
//            if (result.hasTargets())
//            {
//                PhotonTrackedTarget target = result.getBestTarget();
//
//                if (goodTarget == null || target.getPoseAmbiguity() < goodTarget.getPoseAmbiguity())
//                {
//                    goodTarget = target;
//                    bestCam = cams;
//                }
//            }
//        }
//
//        if (goodTarget != null && bestCam != null)
//        {
//            Transform2d camToTarget = goodTarget.getBestCameraToTarget();
//            Translation2d trans = camToTarget.getTranslation();
//            Rotation2d rot = camToTarget.getRotation();
//
//            return new Pose2d(trans, rot);
//        }
//    }
    // public void driveToAprilTags() {
    //     PhotonTrackedTarget goodTarget = null;
    //     PhotonCamera bestCam = null;

    //     for (PhotonCamera cams : cam)
    //     {
    //         var result = cams.getLatestResult();

    //         if (result.hasTargets())
    //         {
    //             PhotonTrackedTarget target = result.getBestTarget();

    //             if (goodTarget == null || target.getPoseAmbiguity() < goodTarget.getPoseAmbiguity())
    //             {
    //                 goodTarget = target;
    //                 bestCam = cams;
    //             }
    //         }
    //     }
    //     if (goodTarget != null && bestCam != null)
    //     {
    //         double yaw = goodTarget.getYaw();
    //         double pitch = goodTarget.getPitch();

    //         double distanceAway = calculateDistanceToTarget(pitch);

    //         double bestX = distanceAway * Math.cos(Math.toRadians(yaw));
    //         double bestY = distanceAway * Math.sin(Math.toRadians(yaw));

    //         double outX = x.calculate(bestX, 0); //Prob tune this val
    //         double outY = y.calculate(bestY, 0); //Prob tune this val
    //         double outTheta = theta.calculate(yaw, 0); //Prob tune this val



    //         //drive.drive(new Translation2d(outX, outY), outTheta, true);  //Chasis moves to calculated distance
    //         drive.drive(null, false);
    //     }
    //     else
    //     {
    //         drive.drive(null, false); //Chasis doesn't move if no targets are seen
    //     }
    // }

    // private double calculateDistanceToTarget(double pitch) {
    //     double camHeight = 0; //TODO get correct val (height from cam)
    //     double tagHeight = 0; //TODO get correct val (actual height from ground)

    //     double angle = Math.toRadians(pitch);

    //     return (tagHeight - camHeight) / Math.tan(angle);
    // }

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

        
    
    
