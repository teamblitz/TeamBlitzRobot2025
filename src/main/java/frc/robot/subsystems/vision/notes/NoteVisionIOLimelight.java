package frc.robot.subsystems.vision.notes;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import frc.lib.util.LimelightHelpers;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

public class NoteVisionIOLimelight implements NoteVisionIO {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private final String name;
    private final Mat homography =
            Calib3d.findHomography(
                    new MatOfPoint2f( // Camera points
                            new Point(177, 134),
                            new Point(469, 132),
                            new Point(532, 270),
                            new Point(132, 266)),
                    new MatOfPoint2f( // Real world points
                            new Point(Units.inchesToMeters(22 + 15), Units.inchesToMeters(-4)),
                            new Point(Units.inchesToMeters(22 + 15), Units.inchesToMeters(5)),
                            new Point(Units.inchesToMeters(52 + 15), Units.inchesToMeters(20)),
                            new Point(Units.inchesToMeters(54 + 15), Units.inchesToMeters(-18))));

    public NoteVisionIOLimelight(String limelightName) {
        this.name = limelightName;
    }

    @Override
    public void updateInputs(NoteVisionInputs inputs) {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);

        inputs.timestampCapture = 0;

        DoubleArrayEntry jsonEntity = LimelightHelpers.getLimelightDoubleArrayEntry(name, "t2d");

        var tsValue = jsonEntity.getAtomic();

        double adjustedTimestamp =
                (tsValue.timestamp / 1000000.0)
                        - ((results.latency_capture + results.latency_pipeline) / 1000.0);

        inputs.timestampCapture = adjustedTimestamp;

        if (results.targets_Detector.length < 1) {
            inputs.valid = false;
            return;
        }
        LimelightHelpers.LimelightTarget_Detector detectorResults = results.targets_Detector[0];

        inputs.tx = -detectorResults.tx;
        inputs.ty = -detectorResults.ty;
        inputs.valid = results.valid;

        inputs.txPixels = detectorResults.tx_pixels;
        inputs.tyPixels = detectorResults.ty_pixels;

        Mat src =
                new MatOfPoint2f(
                        new Point(
                                inputs.txPixels,
                                inputs.tyPixels)); // Matrix of vectors to be transformed
        MatOfPoint2f dst =
                new MatOfPoint2f(); // A mutable matrix for opencv to store thebotspace coordinates;

        try {
            Core.perspectiveTransform(src, dst, homography);
            inputs.projectionValid = true;
        } catch (Exception e) {
            System.out.println("OpenCV Homography transformation failed");
            inputs.projectionValid = false;
        }

        Point noteBotSpace = dst.toArray()[0];

        inputs.botSpaceX = noteBotSpace.x;
        inputs.botSpaceY = noteBotSpace.y;
    }
}
