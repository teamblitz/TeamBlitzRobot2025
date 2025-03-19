package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Vision extends SubsystemBase {
    private final List<PhotonCamera> cam;

    public Vision() {
        this.cam = new ArrayList<>();
        cam.add(new PhotonCamera("1"));  //TODO get correct name
        cam.add(new PhotonCamera("2"));  //TODO get correct name
        cam.add(new PhotonCamera("3"));  //TODO get correct name
        cam.add(new PhotonCamera("4"));  //TODO get correct name
    }

    public Pose2d getAprilTagPose2d() {
        PhotonTrackedTarget goodTarget = null;
        PhotonCamera bestCam = null;

        for (PhotonCamera cams : cam)
        {
            var result = cams.getLatestResult();

            if (result.hasTargets())
            {
                PhotonTrackedTarget target = result.getBestTarget();

                if (goodTarget == null || target.getPoseAmbiguity() < goodTarget.getPoseAmbiguity())
                {
                    goodTarget = target;
                    bestCam = cams;
                }
            }
        }

        if (goodTarget != null && bestCam != null)
        {
            Transform2d camToTarget = goodTarget.getBestCameraToTarget();
            Translation2d trans = camToTarget.getTranslation();
            Rotation2d rot = camToTarget.getRotation();

            return new Pose2d(trans, rot);
        }
    }
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
        boolean hasTargets = false;

        for (PhotonCamera cams : cam)
        {
            if (cams.getLatestResult().hasTargets())
            {
                hasTargets = true;
                break; //maybe?
            }
        }
        SmartDashboard.putBoolean("Cam has targets", hasTargets);

        Pose2d pose = getAprilTagPose2d();

        if (pose != null)
        {
            SmartDashboard.putNumber("Pose X: ", pose.getX());
            SmartDashboard.putNumber("Pose Y: ", pose.getY());
            SmartDashboard.putNumber("Rotation: ", pose.getRotation().getRadians());
        }
        else
        {
            SmartDashboard.putString("Pose: ", "None found");
        }



        // for each new pose
        // drive.addVisionMeasurement(pose, timestamp, ??standard_deviation??)
    }

}

        
    
    
