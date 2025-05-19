package frc.lib.monitor;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.reduxrobotics.canand.CanandDevice;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.util.Capture;

import lombok.Getter;

import java.util.function.BooleanSupplier;

/**
 * A class designed to passively monitor and report device connection issues.
 * Creates 2 NT Alerts, (ActiveHardwareAlerts and StickyHardwareAlerts)
 */
public class HardwareWatchdog {
    @Getter
    public static final HardwareWatchdog instance = new HardwareWatchdog();

    private final EventLoop eventLoop = new EventLoop();

    // String concatenations are slow, and there really is no need to update the alert more
    // frequently
    // This only impacts the while active periodic alert updates, and does not impact when the
    // appearance or disappearance of the alert occurs
    // Or the start/stop times
    private static final double alertUpdatePeriod = .2;

    // Give the hardware a second to connect
    private static final double gracePeriod = 2.0;

    private static final String groupActive = "ActiveHardwareAlerts";
    private static final String groupSticky = "StickyHardwareAlerts";

    private HardwareWatchdog() {
        var timer = new Timer();
        timer.start();

//        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
//            if (timer.get() > gracePeriod) {
//                eventLoop.poll();
//            }
//        });
    }

    public void registerCTREDevice(ParentDevice device, Class<?> parent) {
        registerDevice(
                device::isConnected,
                device.getClass().getSimpleName() + " " + device.getDeviceID(),
                parent);
    }

    public void registerSpark(SparkBase spark, Class<?> parent) {
        registerDevice(
                () -> !spark.getFaults().can && spark.getFirmwareVersion() != 0,
                "SparkMax " + spark.getDeviceId(),
                parent);
    }

    public void registerDutyCycleEncoder(DutyCycleEncoder encoder, Class<?> parent) {
        registerDevice(
                encoder::isConnected, "DutyCycleEncoder " + encoder.getSourceChannel(), parent);
    }

    public void registerReduxDevice(CanandDevice device, Class<?> parent) {
        registerDevice(
                device::isConnected,
                device.getClass().getSimpleName() + " " + device.getAddress().getDeviceId(),
                parent);
    }

    public void registerDevice(BooleanSupplier isConnected, String name, Class<?> parent) {
        String deviceDescriptor =
                String.format("\"%s\" belonging to \"%s\"", name, parent.getSimpleName());

        Capture<Alert> alert = new Capture<>(
                new Alert(groupActive, "deviceDisconnectAlert", Alert.AlertType.kWarning));
        Capture<Alert> stickyAlert = new Capture<>(null);

        Capture<Double> disconnectedAt = new Capture<>(-1.0);

        var updateTimer = new Timer();

        // On disconnect
        new Trigger(eventLoop, isConnected)
                .negate()
                .whileTrue(Commands.startRun(
                                () -> {
                                    updateTimer.restart();
                                    disconnectedAt.inner = Timer.getTimestamp();

                                    // Create a new sticky alert, this way old ones persist
                                    stickyAlert.inner = new Alert(
                                            groupSticky,
                                            "deviceDisconnectStickyAlert",
                                            Alert.AlertType.kWarning);

                                    alert.inner.set(true);
                                    stickyAlert.inner.set(true);
                                },
                                () -> {
                                    if (updateTimer.advanceIfElapsed(alertUpdatePeriod)) {
                                        var timeDisconnected =
                                                Timer.getTimestamp() - disconnectedAt.get();

                                        var text = String.format(
                                                "Device %s has been disconnected for %.2f seconds (since %.2f)",
                                                deviceDescriptor,
                                                timeDisconnected,
                                                disconnectedAt.get());

                                        alert.inner.setText(text);
                                        stickyAlert.inner.setText(text);
                                    }
                                })
                        .finallyDo(() -> {
                            alert.get().set(false);

                            var stickyText = String.format(
                                    "Device %s was disconnected for %.2f seconds (from %.2f to %.2f)",
                                    deviceDescriptor,
                                    Timer.getTimestamp() - disconnectedAt.get(),
                                    disconnectedAt.get(),
                                    Timer.getTimestamp());

                            stickyAlert.inner.setText(stickyText);
                            updateTimer.stop();
                        })
                        .ignoringDisable(true));
    }
}
