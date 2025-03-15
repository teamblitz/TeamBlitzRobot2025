package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends BlitzSubsystem {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake(IntakeIO io) {
        super("intake");

        this.io = io;

        new Trigger(() -> intakeState == IntakeState.Feeding && coralState == CoralState.Indexed)
                .whileTrue(
                        Commands.sequence(
                                Commands.waitUntil(() -> !intakeSensor()),
                                Commands.run(() -> coralState = CoralState.Empty)));
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    public boolean hasCoral() {
        return coralState == CoralState.Indexed || coralState == coralState.Unindexed;
    }

    public Command handoff() {
        return startEnd(() -> io.setSpeed(HANDOFF_SPEED), () -> io.setSpeed(0))
                .until(this::intakeSensor)
                .onlyIf(() -> !intakeSensor());
    }

    public Command reverse() {
        return startEnd(() -> io.setSpeed(REVERSE_SPEED), () -> io.setSpeed(0));
    }

    public Command algae_eject() {
        return startEnd(() -> io.setSpeed(ALGAE_REMOVAL), () -> io.setSpeed(0));
    }

    public Command shoot_coral() {
        return startEnd(() -> io.setSpeed(SHOOT_CORAL), () -> io.setSpeed(0));
    }

    private Command stop() {
        return runOnce(() -> io.setSpeed(0));
    }

    private boolean intakeSensor() {
        return inputs.breakBeam;
    }

    private Command setSpeedCommand(double speed) {
        return startEnd(() -> io.setSpeed(speed), this::stop);
    }

    public enum CoralState {
        Indexed,
        Unindexed,
        Empty,
        Unknown
    }

    private CoralState coralState = CoralState.Indexed;

    public enum IntakeState {
        Intaking,
        Feeding,
        Ejecting,
        Indexing,
        Idle,
        Manual
    }

    private IntakeState intakeState = IntakeState.Idle;

    public Command indexIntake() {
        return Commands.none();
    }

    public Command autoIndex() {
        return new ConditionalCommand(
                indexIntake(), Commands.none(), () -> coralState == CoralState.Unindexed);
    }
}
