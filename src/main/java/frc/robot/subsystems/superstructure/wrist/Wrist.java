package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends BlitzSubsystem {
    public final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private final SysIdRoutine routine;

    public Wrist(WristIO io) {
        super("wrist");
        this.io = io;

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("Characterization");

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(null, Units.Volts.of(5), null),
                        new SysIdRoutine.Mechanism((volts) -> volts.in(Units.Volts), null, this));

        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .withName("Wrist Quasistic Forward"));
        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .withName("Wrist Quasistic Reverse"));

        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kForward).withName("Wrist Dynamic Forward"));
        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("Wrist Dynamic Reverse"));
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    final TrapezoidProfile wristTrapezoid =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(0.8, 1.6));

    //    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(double position, double
    // velocity);
    //    private TrapezoidProfile.State m_setPoint = new TrapezoidProfile.State(double position,
    // double velocity);
    //
    public Command setSpeed(double speed) {
        return setSpeed(() -> speed);
    }

    public Command setSpeed(DoubleSupplier speed) {

        return runEnd(
                        () -> {
                            io.setPercent(speed.getAsDouble());
                        },
                        () -> {
                            io.setPercent(0);
                        })
                .withName(logKey + "/speed");
    }

    public Command r1Rotation() {

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
            throw new UnsupportedOperationException("Not supported yet. <- DO NOT REMOVE");
        }
        return run(
                () -> {
                    io.setSetpoint(Constants.Wrist.wristRotations.r1RotationValue, 0, 0);
                });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
