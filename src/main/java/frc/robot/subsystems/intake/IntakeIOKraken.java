package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.lib.monitor.HardwareWatchdog;

import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicBoolean;

public class IntakeIOKraken implements IntakeIO {
    public final TalonFX intake;

    private final DigitalInput breakBeam;
    private final AsynchronousInterrupt coralInterrupt;

    public IntakeIOKraken() {
        intake = new TalonFX(CAN_ID); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);
        breakBeam = new DigitalInput(0);

        intake.getConfigurator().apply(config);

        ControlRequest _threadInterruptStop = new NeutralOut().withUpdateFreqHz(0);

        coralInterrupt = new AsynchronousInterrupt(breakBeam, (rising, falling) -> {
            if (falling && doInterrupt.get()) {
                System.out.println("STOPPINGS **********");
                intake.setControl(_threadInterruptStop);
            }
        });

        // Current sensor wiring has reversed phase for some reason.
        coralInterrupt.setInterruptEdges(false, true);

        HardwareWatchdog.getInstance().registerCTREDevice(intake, this.getClass());
    }

    @Override
    public void setSpeed(double speed) {
        intake.set(speed);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.breakBeam = !breakBeam.get();

        Logger.recordOutput("intake/lastInterruptTriggered", coralInterrupt.getFallingTimestamp());
    }

    private final AtomicBoolean doInterrupt = new AtomicBoolean(false);

    @Override
    public void enableCoralInterrupt(boolean interrupt) {
        doInterrupt.set(true);
    }
}
