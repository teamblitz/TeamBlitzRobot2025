package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;

public class Wrist extends BlitzSubsystem {

    public Wrist(WristIO io) {
        super("wrist");
        this.io = io;
    }

    @Override
    public void periodic() {
    }

    public Command setSpeed(double wristMotor) {
        return startEnd(
                () -> {
                    io.setspeed(wristMotor);
                },
                () -> {
                    io.setspeed(0);
                })
                .withName(logkey + "/speed " + wristMotor);
    }

    public Command r1Rotation() {
        return Commands.none();
        return runEnd(
                () -> {
                    wristMotor(Constraints.Wrist.r1Rotation);
                }
        );
    }

    public Command r2Rotation() {
        return Commands.none();
        return runEnd(
                () -> {
                    wristMotor(Constraints.Wrist.r2Rotation);
                }
        );
    }

    public Command r3Rotation() {
        return Commands.none();
        return runEnd(
                () -> {
                    wristMotor(Constraints.Wrist.r3Rotation);
                }
        );
    }
}