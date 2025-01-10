package frc.robot.subsystems.drive.range;

import static frc.robot.Constants.Drive.*;

import com.playingwithfusion.TimeOfFlight;

public class RangeSensorIOFusion implements RangeSensorIO {

    TimeOfFlight rangeSensor;

    public RangeSensorIOFusion() {
        rangeSensor = new TimeOfFlight(FUSION_TIME_OF_FLIGHT_ID);
        // rangeSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 50);
    }

    @Override
    public void updateInputs(RangeSensorIOInputs inputs) {
        inputs.range = rangeSensor.getRange() * 1e-3;
        inputs.stdRange = rangeSensor.getRangeSigma() * 1e-3;
        inputs.ambientLight = rangeSensor.getAmbientLightLevel();
        inputs.mode = rangeSensor.getRangingMode().toString();
        inputs.sampleTime = rangeSensor.getSampleTime();

        inputs.valid = rangeSensor.getStatus() == TimeOfFlight.Status.Valid;
        inputs.status = rangeSensor.getStatus().toString();
    }
}
