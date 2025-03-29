package frc.robot.subsystems.climber;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.SupplierUtils;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Climber extends BlitzSubsystem {
    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    private final LoggedTunableNumber deployPosition =
            new LoggedTunableNumber("climber/deployPosition", Math.toDegrees(DEPLOY_POSITION));
    private final LoggedTunableNumber climbPosition =
            new LoggedTunableNumber("climber/climbPosition", Math.toDegrees(CLIMB_POSITION));
    private final LoggedTunableNumber restowPosition =
            new LoggedTunableNumber("climber/restowPosition", Math.toDegrees(RESTOW_POSITION));

    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("climber/unloaded_kP", UnloadedGains.KP);
    //    private final LoggedTunableNumber unloadedMaxVel = new
    // LoggedTunableNumber("climber/unloaded_max_vel", MAX);

    private final SysIdRoutine routine;

    public enum State {
        DEPLOYED,
        CLIMB,
        RESTOWED,
        STOWED
    }

    @Getter private State state = State.STOWED;

    public Climber(ClimberIO io) {
        super("climber");
        this.io = io;

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.per(Units.Seconds).of(1),
                                Units.Volts.of(5),
                                null,
                                Constants.compBot()
                                        ? (state) ->
                                                SignalLogger.writeString(
                                                        "sysid-climber-state", state.toString())
                                        : null),
                        new SysIdRoutine.Mechanism(
                                (volts) -> io.setVolts(volts.in(Units.Volts)), null, this));

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("characterization/climber");

        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .withName("Climber Quasistic Forward"));
        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .withName("Climber Quasistic Reverse"));

        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kForward).withName("Climber Dynamic Forward"));
        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("Climber Dynamic Reverse"));

        //        ShuffleboardTab climbTab = Shuffleboard.getTab("climber");
        //        climbTab.add(deployClimber());
        //        climbTab.add(restowClimber());
        //        climbTab.add(climb());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(logKey + "/state", state.toString());
    }

    private Command goToPosition(DoubleSupplier position) {
        return new FunctionalCommand(
                () -> io.setMotionProfile(position.getAsDouble()),
                () -> {},
                (interrupted) -> {
                    if (interrupted) io.setSpeed(0);
                },
                () -> MathUtil.isNear(position.getAsDouble(), inputs.position, EPSILON),
                this);
    }

    public Command deployClimber() {
        return goToPosition(SupplierUtils.toRadians(deployPosition))
                .andThen(() -> state = State.DEPLOYED)
                .withName(logKey + "/deployClimber");
    }

    public Command climb() {
        return goToPosition(SupplierUtils.toRadians(climbPosition))
                .alongWith(Commands.run(() -> state = State.CLIMB))
                .withName(logKey + "/climb");
    }

    public Command restowClimber() {
        return goToPosition(SupplierUtils.toRadians(restowPosition))
                .andThen(() -> state = State.RESTOWED)
                .withName(logKey + "/restowClimber");
    }

    public Command setSpeed(double speed) {
        return runEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrakeMode(false), () -> io.setBrakeMode(true))
                .beforeStarting(() -> Leds.getInstance().superstructureCoast = true)
                .finallyDo(() -> Leds.getInstance().superstructureCoast = false)
                .ignoringDisable(true)
                .withName(logKey + "/coast");
    }
}
