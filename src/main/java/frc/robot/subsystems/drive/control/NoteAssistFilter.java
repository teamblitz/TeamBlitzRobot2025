package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.notes.NoteVisionIO;
import java.util.Optional;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.littletonrobotics.junction.Logger;

public class NoteAssistFilter extends ChassisSpeedFilter {

    //    private final Supplier<Translation2d> notePose;
    private final NoteVisionIO.NoteVisionInputs noteVisionInputs;

    private Debouncer detectedNoteDebounce = new Debouncer(5 / 30.0, Debouncer.DebounceType.kBoth);

    private final ProfiledPIDController rotateToHeadingPid;

    public NoteAssistFilter(Drive drive, NoteVisionIO.NoteVisionInputs noteVisionInputs) {
        super(drive, true);
        this.noteVisionInputs = noteVisionInputs;

        rotateToHeadingPid =
                new ProfiledPIDController(.1, 0, 0, new TrapezoidProfile.Constraints(90, 180));
        rotateToHeadingPid.enableContinuousInput(-180, 180);
        rotateToHeadingPid.setTolerance(2);
    }

    @Override
    public ChassisSpeeds apply(ChassisSpeeds initialSpeeds) {
        Logger.recordOutput("vision/note/test1", true);
        if (!detectedNoteDebounce.calculate(
                noteVisionInputs.projectionValid && noteVisionInputs.valid)) return initialSpeeds;

        double timestamp = noteVisionInputs.timestampCapture;

        // Find out where the robot was when we saw the note
        if (Timer.getFPGATimestamp() - timestamp > 1) {
            System.out.println(
                    "NoteAssist: LL Note Position too latent! expected less than 1s latency, got "
                            + (Timer.getFPGATimestamp() - timestamp)
                            + " seconds. Ignoring reading!");
            return initialSpeeds;
        }

        Optional<Pose2d> oldPoseSample =
                drive.samplePreviousPose(noteVisionInputs.timestampCapture);
        if (oldPoseSample.isEmpty()) {
            System.out.println("NoteAssist: No old pose data for timestamp!");
            return initialSpeeds;
        }

        Logger.recordOutput("vision/note/oldPoseSample", oldPoseSample.get());

        double botSpaceXLatent = noteVisionInputs.botSpaceX;
        double botSpaceYLatent = noteVisionInputs.botSpaceY;

        Translation2d notePoseField =
                oldPoseSample
                        .get()
                        .transformBy(
                                new Transform2d(botSpaceXLatent, botSpaceYLatent, new Rotation2d()))
                        .getTranslation();

        Logger.recordOutput("vision/note/notePoseField", notePoseField);

        Translation2d fieldRelativeNoteBotSpace =
                notePoseField.minus(drive.getPose().getTranslation());

        Logger.recordOutput("vision/note/fieldRelativeNoteBotSpace", fieldRelativeNoteBotSpace);

        Vector2D notePosition =
                new Vector2D(fieldRelativeNoteBotSpace.getX(), fieldRelativeNoteBotSpace.getY());

        if (notePosition.getNorm() == 0) return initialSpeeds;

        Vector2D noteDirection = notePosition.normalize();

        Vector2D driverVelocity =
                new Vector2D(initialSpeeds.vxMetersPerSecond, initialSpeeds.vyMetersPerSecond);
        //        Vector2D driverDirection = driverVelocity.normalize();

        Logger.recordOutput("vision/note/driver/x", driverVelocity.getX());
        Logger.recordOutput("vision/note/driver/y", driverVelocity.getY());

        Rotation2d noteAngle = new Rotation2d(noteDirection.getX(), noteDirection.getY());
        Rotation2d driverAngle = new Rotation2d(driverVelocity.getX(), driverVelocity.getY());

        Rotation2d adjustedAngle =
                driverAngle.plus(
                        noteAngle
                                .minus(driverAngle)
                                .times(
                                        Constants.Drive.NoteAssist.ACTIVATION_FUNCTION
                                                .applyAsDouble(
                                                        noteAngle
                                                                .minus(driverAngle)
                                                                .getRadians())));

        Vector2D adjustedDirection = new Vector2D(adjustedAngle.getCos(), adjustedAngle.getSin());

        Vector2D adjustedVelocity =
                adjustedDirection.scalarMultiply(driverVelocity.dotProduct(adjustedDirection));

        Logger.recordOutput("noteAssist/noteAngle", noteAngle.getDegrees());
        Logger.recordOutput("noteAssist/driverAngle", driverAngle.getDegrees());
        Logger.recordOutput("noteAssist/adjustedAngle", adjustedAngle.getDegrees());
        Logger.recordOutput(
                "vision/note/botpos/angle", fieldRelativeNoteBotSpace.getAngle().getDegrees());

        double omegaHeading =
                rotateToHeadingPid.calculate(
                        drive.getYaw().getDegrees(),
                        new TrapezoidProfile.State(
                                fieldRelativeNoteBotSpace.getAngle().getDegrees(), 0));

        if (Math.abs(noteAngle.minus(driverAngle).getRadians())
                > Constants.Drive.NoteAssist.ACTIVATION_RANGE) {
            if (Math.abs(drive.getYaw().minus(noteAngle).getRadians())
                    < Constants.Drive.NoteAssist.ACTIVATION_RANGE) {
                return new ChassisSpeeds(
                        initialSpeeds.vxMetersPerSecond,
                        initialSpeeds.vyMetersPerSecond,
                        omegaHeading);
            } else return initialSpeeds;
        }

        return new ChassisSpeeds(adjustedVelocity.getX(), adjustedVelocity.getY(), omegaHeading);
    }

    @Override
    public void reset() {
        rotateToHeadingPid.reset(drive.getYaw().getDegrees());
    }
}
