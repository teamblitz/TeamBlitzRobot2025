package frc.robot.subsystems.drive.range;

import org.littletonrobotics.junction.AutoLog;

public interface RangeSensorIO {
    @AutoLog
    public class RangeSensorIOInputs {
        public double range;
        public double stdRange;
        public double ambientLight;
        public boolean valid;
        public String status;

        public String mode;
        public double sampleTime;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(RangeSensorIOInputs inputs) {}
}
