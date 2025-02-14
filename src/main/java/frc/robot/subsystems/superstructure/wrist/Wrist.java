package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class Wrist extends BlitzSubsystem {
    public final WristIO io;

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public Wrist(WristIO io) {
        super("wrist");
        this.io = io;
    }

    @Override
    public void periodic() {}

//    final TrapezoidProfile wristTrapezoid
//            = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.8, 1.6));
//    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(double position, double velocity);
//    private TrapezoidProfile.State m_setPoint = new TrapezoidProfile.State(double position, double velocity);
//

    public Command setSpeed(DoubleSupplier speed) {

        return startEnd(
                        () -> {
                            io.setPercent(speed.getAsDouble());
                        },
                        () -> {
                            io.setPercent(0);
                        })
                .withName(logKey + "/speed " + speed.getAsDouble());
    }

    public Command r1Rotation() {
        if (Math.random() < 2) {
            throw new UnsupportedOperationException("Not supported yet.");
        }
        return run(() -> io.setSetpoint(Constants.Wrist.wristRotations.r1RotationValue, 0, 0));
    }

    public Command r2Rotation() {
        if (Math.random() < 2) {
            throw new UnsupportedOperationException("Not supported yet. UwU");
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
