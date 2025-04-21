package frc.lib.monitor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.MutableReference;
import frc.lib.util.Capture;

import java.util.function.BooleanSupplier;

public class HardwareWatchdog {
    
    public void registerMotor(TalonFX talon, Class<?> parent) {
        registerDeviceConnection(talon::isConnected, talon.getDescription(), parent);
    }

    public void registerMotor(SparkMax spark, Class<?> parent) {
        registerDeviceConnection(() -> spark.getFaults().can || spark.getFirmwareVersion() == 0, "SparkMax " + spark.getDeviceId(), parent);
    }

    private static final String group = "HardwareAlerts";
    private static final String stickyGroup = "StickyAlerts";


    public static void registerDeviceConnection(BooleanSupplier connected, String name, Class<?> parent) {
        String deviceDescriptor = String.format("\"%s\" belonging to \"%s\"", name, parent.getSimpleName());

        Capture<Alert> alert = new Capture<>(new Alert(group, "deviceDisconnectAlert", Alert.AlertType.kWarning));
        Capture<Alert> stickyAlert = new Capture<>(null);

        Capture<Double> disconnectedAt = new Capture<>(-1.0);

        // On disconnect
        new Trigger(connected).negate().whileTrue(
                Commands.startRun(() -> {
                    disconnectedAt.inner = Timer.getTimestamp();

                    // Create a new sticky alert
                    stickyAlert.inner = new Alert(stickyGroup, "deviceDisconnectStickyAlert", Alert.AlertType.kWarning);

                    alert.inner.set(true);
                    stickyAlert.inner.set(true);
                }, () -> {
                    var timeDisconnected = Timer.getTimestamp() - disconnectedAt.get();

                    var text = String.format(
                            "Device %s has been disconnected for %.2f seconds (since %.2f)",
                            deviceDescriptor, timeDisconnected, disconnectedAt.get()
                    );

                    alert.inner.setText(text);
                    stickyAlert.inner.setText(text);

                }).finallyDo(() ->
                        {
                            alert.get().set(false);

                            var stickyText = String.format(
                                    "Device %s was disconnected for %.2f seconds (from %.2f to %.2f)",
                                    deviceDescriptor, Timer.getTimestamp() - disconnectedAt.get(), disconnectedAt.get(), Timer.getTimestamp()
                            );

                            stickyAlert.inner.setText(stickyText);
                        })
                        .ignoringDisable(true)
        );
    }
}
