package frc.robot.subsystems.winch;

import static frc.robot.Constants.Winch.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import frc.lib.math.EqualsUtil;
import org.littletonrobotics.junction.Logger;

public class Winch extends BlitzSubsystem {
    private final WinchIO io;
    // TODO Make Auto Logs Work
    private final WinchInputsAutoLogged inputs = new WinchInputsAutoLogged();

    public Winch(WinchIO io) {
        super("winch");
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    public Command raiseFunnel() {
        return runOnce(() -> io.setMotionProfile(FUNNEL_UP_POSITION))
                .andThen(
                        Commands.waitUntil(
                                () ->
                                        EqualsUtil.epsilonEquals(
                                                inputs.position, FUNNEL_UP_POSITION, 1e-3)));
    }

    public Command lowerFunnel() {
        return runOnce(() -> io.setMotionProfile(FUNNEL_UP_POSITION))
                .andThen(
                        Commands.waitUntil(
                                () ->
                                        EqualsUtil.epsilonEquals(
                                                inputs.position, FUNNEL_DOWN_POSITION, 1e-3)));
    }
}
