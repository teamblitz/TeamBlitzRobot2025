package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOOrangePi implements Vision {

    public VisionIOOrangePi() {
        PhotonCamera cam1_20 = new PhotonCamera(".20 Cam1");
        PhotonCamera cam2_20 = new PhotonCamera(".20 Cam2");
        PhotonCamera cam1_21 = new PhotonCamera(".21 Cam1");
        PhotonCamera cam2_21 = new PhotonCamera(".22 Cam2");

        var result1 = cam1_20.getLatestResult();
        var result2 = cam2_20.getLatestResult();
        var result3 = cam1_21.getLatestResult();
        var result4 = cam2_21.getLatestResult();

        boolean hasTargets1 = result1.hasTargets();
        boolean hasTargets2 = result2.hasTargets();
        boolean hasTargets3 = result3.hasTargets();
        boolean hasTargets4 = result4.hasTargets();

        List<PhotonTrackedTarget> target1 = result1.getTargets();
        List<PhotonTrackedTarget> target2 = result2.getTargets();
        List<PhotonTrackedTarget> target3 = result3.getTargets();
        List<PhotonTrackedTarget> target4 = result4.getTargets();

        PhotonTrackedTarget t1 = result1.getBestTarget();
        PhotonTrackedTarget t2 = result2.getBestTarget();
        PhotonTrackedTarget t3 = result3.getBestTarget();
        PhotonTrackedTarget t4 = result4.getBestTarget();


    }

    public void updateInputs() {

    }
}