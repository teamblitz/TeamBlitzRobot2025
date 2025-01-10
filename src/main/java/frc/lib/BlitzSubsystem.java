package frc.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BlitzSubsystem extends SubsystemBase {
    protected final String logKey;

    public BlitzSubsystem(String logKey) {
        this.logKey = logKey;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                logKey + "/defaultCommand",
                getDefaultCommand() != null ? getDefaultCommand().getName() : "none");
        Logger.recordOutput(
                logKey + "/currentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
    }
}
