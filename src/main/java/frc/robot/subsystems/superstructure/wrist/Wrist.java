package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.math.EqualsUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import java.util.function.DoubleSupplier;

import lombok.extern.java.Log;
import org.littletonrobotics.junction.Logger;

public class Wrist extends BlitzSubsystem {
    public final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private TrapezoidProfile profile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(0.8, 1.6));

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private final SysIdRoutine routine;

    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("wrist/kP", Constants.Wrist.WristGains.KP);
    private final LoggedTunableNumber kI =
            new LoggedTunableNumber("wrist/kI", Constants.Wrist.WristGains.KP);
    private final LoggedTunableNumber kD =
            new LoggedTunableNumber("wrist/kD", Constants.Wrist.WristGains.KD);

    private final LoggedTunableNumber kS =
            new LoggedTunableNumber("wrist/kS", Constants.Wrist.WristGains.KS);
    private final LoggedTunableNumber kV =
            new LoggedTunableNumber("wrist/kV", Constants.Wrist.WristGains.KV);
    private final LoggedTunableNumber kA =
            new LoggedTunableNumber("wrist/kA", Constants.Wrist.WristGains.KA);
    private final LoggedTunableNumber kG =
            new LoggedTunableNumber("wrist/kG", Constants.Wrist.WristGains.KG);

    public Wrist(WristIO io) {
        super("wrist");
        this.io = io;

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("Characterization");

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(null, Units.Volts.of(5), null),
                        new SysIdRoutine.Mechanism((volts) -> io.setVolts(volts.in(Units.Volts)), null, this));

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

        Logger.recordOutput(
                logKey + "/absEncoderDegrees", Math.toRadians(inputs.absoluteEncoderPosition));
        
        LoggedTunableNumber.ifChanged(
                hashCode(), pid -> io.setPid(pid[0], pid[1], pid[2]), kP, kI, kD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                (kSGVA) -> io.setFF(kSGVA[0], kSGVA[1], kSGVA[2], kSGVA[3]),
                kS,
                kG,
                kV,
                kA);
    }

    public double getPosition() {
        return inputs.positionRadians;
    }

    public double getVelocity() {
        return inputs.velocityRadiansPerSecond;
    }

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

    public Command withGoal(TrapezoidProfile.State goal) {
        return runOnce(
                        () -> {
                            this.goal = goal;
                        })
                .until(() -> setpoint.equals(goal))
                .handleInterrupt(() -> this.goal = setpoint)
                .beforeStarting(refreshCurrentState());
    }

    private Command refreshCurrentState() {
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
                .onlyIf(
                        () ->
                                setpoint == null
                                        || !EqualsUtil.epsilonEquals(
                                                setpoint.position, getPosition(), .02)
                                        || !EqualsUtil.epsilonEquals(
                                                setpoint.velocity, getVelocity(), 0.04));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrakeMode(false), () -> io.setBrakeMode(true))
                .beforeStarting(() -> Leds.getInstance().armCoast = true)
                .finallyDo(() -> Leds.getInstance().armCoast = false)
                .ignoringDisable(true)
                .withName(logKey + "/coast");
    }
}
