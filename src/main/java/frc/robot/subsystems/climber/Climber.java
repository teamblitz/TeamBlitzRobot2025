package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Maybe divide this into 2 subsystems, depends on how we want to control it. The current way we do
 * this, 2 subsystems is ideal (and is kinda what we are pseudo doing)
 */
public class Climber extends BlitzSubsystem {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private ElevatorFeedforward ffLoadedLeft;
    private ElevatorFeedforward ffLoadedRight;
    private ElevatorFeedforward ffUnLoadedLeft;
    private ElevatorFeedforward ffUnLoadedRight;

    public Climber(ClimberIO io) {
        super("climber");
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    public Command setSpeed(double left, double right) {
        return startEnd(
                        () -> {
                            io.setSpeedLeft(left);
                            io.setSpeedRight(right);
                        },
                        () -> {
                            io.setSpeedLeft(0);
                            io.setSpeedRight(0);
                        })
                .withName(logKey + "/speed " + left + " " + right);
    }

    public Command climb() {
        return runEnd(
                        () -> {
                            io.setMotionMagicLeft(0);
                            io.setMotionMagicRight(0);
                        },
                        () -> {
                            io.setMotionMagicLeft(inputs.positionLeft);
                            io.setMotionMagicRight(inputs.positionRight);
                        })
                .withName(logKey + "/climb");
    }

    public Command goUp() {
        return runEnd(
                        () -> {
                            io.setMotionMagicLeft(Constants.Climber.MAX_EXTENSION);
                            io.setMotionMagicRight(Constants.Climber.MAX_EXTENSION);
                        },
                        () -> {
                            io.setSpeedLeft(0);
                            io.setSpeedRight(0);
                        })
                .until(
                        () ->
                                MathUtil.isNear(
                                                Constants.Climber.MAX_EXTENSION,
                                                inputs.positionLeft,
                                                .005)
                                        && MathUtil.isNear(
                                                Constants.Climber.MAX_EXTENSION,
                                                inputs.positionRight,
                                                .005))
                .withName(logKey + "/climbersUp");
    }
}
