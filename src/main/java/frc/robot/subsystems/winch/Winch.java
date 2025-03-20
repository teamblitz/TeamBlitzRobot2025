package frc.robot.subsystems.winch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Winch.*;

public class Winch extends BlitzSubsystem {
    private final WinchIO io;
    private final WinchInputsAutoLogged inputs = new WinchInputsAutoLogged();

    private final LoggedTunableNumber maxVelocity =
            new LoggedTunableNumber("winch/maxVelocity", MAX_VELOCITY);
    private final LoggedTunableNumber maxAccel =
            new LoggedTunableNumber("winch/maxAccel", MAX_ACCEL);
    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("winch/kP", KP);

    private final LoggedTunableNumber matchFunnelUp =
            new LoggedTunableNumber("winch/matchFunnelUp", MATCH_FUNNEL_UP);

    private final LoggedTunableNumber matchFunnelDown =
        new LoggedTunableNumber("winch/matchFunnelDown", MATCH_FUNNEL_DOWN);

    private final LoggedTunableNumber pitFunnelStow =
            new LoggedTunableNumber("winch/pitFunnelStow", PIT_FUNNEL_STOW);


    public Winch(WinchIO io) {
        super("winch");
        this.io = io;

        ShuffleboardTab winchTab = Shuffleboard.getTab("winch");
        winchTab.add(raiseFunnel());
        winchTab.add(lowerFunnel());
        winchTab.add(pitFunnelReady());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        LoggedTunableNumber.ifChanged(
                hashCode(), p -> io.setPid(p[0], 0, 0), kP);

        LoggedTunableNumber.ifChanged(
                hashCode(), VA -> io.setMotionProfilePrams(VA[0], VA[1]), maxVelocity, maxAccel
        );

    }

    public Command manualUp() {
        return runEnd(() -> io.setSpeed(.25), () -> io.setSpeed(0));
    }

    public Command manualDown() {
        return runEnd(() -> io.setSpeed(-.25), () -> io.setSpeed(0));
    }

    private Command goToPosition(DoubleSupplier position) {
        return runEnd(() -> io.setMotionProfile(position.getAsDouble()), () -> io.setSpeed(0))
                .andThen(
                        Commands.waitUntil(
                                () ->
                                        MathUtil.isNear(
                                                position.getAsDouble(), inputs.position, EPSILON)));
    }

    // Designed for match use
    public Command raiseFunnel() {
        return goToPosition(matchFunnelUp).withName(logKey + "/raiseFunnel");
    }

    // For match use
    public Command lowerFunnel() {
        return goToPosition(matchFunnelDown).withName(logKey + "/lowerFunnel");
    }

    public Command pitFunnelReady() {
        return Commands.runOnce(() -> io.setPosition(0)).andThen(
                goToPosition(pitFunnelStow).withName(logKey + "/pitFunnelReady"));
    }
}
