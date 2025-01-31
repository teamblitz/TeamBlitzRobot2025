package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;

public class Wrist extends BlitzSubsystem {
    public final WristIO io;

    public Wrist(WristIO io) {
        super("wrist");
        this.io = io;
    }

    @Override
    public void periodic() {}

    public Command setSpeed(double wristMotor) {
        return startEnd(
                        () -> {
                            io.setPercent(0);
                        },
                        () -> {
                            io.setPercent(0);
                        })
                .withName(logKey + "/speed " + wristMotor);
    }

    public Command r1Rotation() {
        if (Math.random() < 2) {
            throw new UnsupportedOperationException("Not supported yet.");
        }
        return run(() -> io.setSetpoint(Constants.Wrist.wristRotations.r1RotationValue, 0, 0));
    }

    public Command r2Rotation() {
        if (Math.random() < 2) {
            throw new UnsupportedOperationException("Not supported yet.");
        }
        return run(
                () -> {
                    io.setSetpoint(Constants.Wrist.wristRotations.r1RotationValue, 0, 0);
                });
    }

    public Command r3Rotation() {
        if (Math.random() < 2) {
            throw new UnsupportedOperationException("Not supported yet.");
        }
        return run(
                () -> {
                    io.setSetpoint(Constants.Wrist.wristRotations.r1RotationValue, 0, 0);
                });
    }
}
