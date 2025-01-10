package frc.robot.subsystems.vision.notes;

import org.littletonrobotics.junction.AutoLog;

public interface NoteVisionIO {
    @AutoLog
    public class NoteVisionInputs {
        public double tx;
        public double ty;

        public double txPixels;
        public double tyPixels;

        public double botSpaceX;
        public double botSpaceY;

        public boolean valid;
        public boolean projectionValid;

        public double timestampCapture;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(NoteVisionIO.NoteVisionInputs inputs) {}
}
