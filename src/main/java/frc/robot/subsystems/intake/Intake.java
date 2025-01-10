package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.leds.Leds;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends BlitzSubsystem {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    BooleanSupplier manualOverride;

    public Intake(IntakeIO io, BooleanSupplier manualOverride) {
        super("intake");

        this.io = io;
        this.manualOverride = manualOverride;
        setDefaultCommand(automaticIndex());

        //         Goals updating
        new Trigger(() -> intakeState == IntakeState.Feeding && noteState == NoteState.Indexed)
                .whileTrue(
                        Commands.sequence(
                                Commands.waitUntil(() -> !intakeSensor()),
                                Commands.run(() -> noteState = NoteState.Empty)));
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);

        // Update LEDs
        Leds.getInstance().hasNote = hasNote();
        Leds.getInstance().indexing = intakeState == IntakeState.Indexing;

        Logger.recordOutput("Intake/NoteState", noteState.name());
        Logger.recordOutput("Intake/IntakeState", intakeState.name());
    }

    private boolean intakeSensor() {
        return inputs.breakBeam;
    }

    public void intake() {
        io.set(0.7);
    } // was .7

    public void eject() {
        io.set(-0.3);
    }

    public void stop() {
        io.set(0);
    }

    public Command intakeGroundAutomatic(double speed) {
        return Commands.race(
                        setSpeedCommand(speed)
                                .until(() -> inputs.breakBeam)
                                .andThen(() -> noteState = NoteState.Indexed)
                                .onlyIf(
                                        () ->
                                                !inputs.breakBeam
                                                        && intakeState != IntakeState.Indexing),
                        Commands.startEnd(
                                () -> intakeState = IntakeState.Intaking,
                                () -> intakeState = IntakeState.Idle))
                .onlyIf(() -> intakeState != IntakeState.Indexing)
                .withName(logKey + "/automaticIntake");
    }

    public Command intakeGroundAutomatic() {
        return intakeGroundAutomatic(.7);
    }

    public Command feedShooter(double speed) {
        return Commands.parallel(
                        setSpeedCommand(speed),
                        Commands.startEnd(
                                () -> intakeState = IntakeState.Feeding,
                                () -> intakeState = IntakeState.Idle))
                .withName(logKey + "/feed");
    }

    public Command feedShooter() {
        return feedShooter(.4);
    }

    /** Note, should only after intakeCommandSmart finishes */
    public Command indexIntake() {

        return setSpeedCommand(-.15)
                .raceWith(
                        Commands.waitSeconds(0).andThen(Commands.waitUntil(() -> inputs.breakBeam)))
                //                .onlyIf(() -> !inputs.breakBeam)
                .beforeStarting(() -> intakeState = IntakeState.Indexing)
                .finallyDo(
                        () -> {
                            intakeState = IntakeState.Idle;
                            noteState = NoteState.Indexed;
                        })
                // .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .withName(logKey + "/index");
    }

    public Command automaticIndex() {
        return new ConditionalCommand(
                        indexIntake(), Commands.none(), () -> noteState == NoteState.Unindexed)
                .withName(logKey + "/autoIndex");
    }

    public Command ejectCommand() {
        return ejectCommand(-.3);
    }

    public Command ejectCommand(double speed) {
        return Commands.parallel(
                        setSpeedCommand(-Math.abs(speed)),
                        Commands.startEnd(
                                () -> intakeState = IntakeState.Ejecting,
                                () -> intakeState = IntakeState.Idle))
                .finallyDo(() -> noteState = NoteState.Empty)
                .withName(logKey + "/eject");
    }

    private Command setSpeedCommand(double speed) {
        return startEnd(() -> io.set(speed), this::stop);
    }

    public enum NoteState {
        Indexed,
        Unindexed,
        Empty,
        Unknown
    }

    private NoteState noteState =
            NoteState.Indexed; // We start the match with the note in an indexed state

    public enum IntakeState {
        Intaking,
        Feeding,
        Ejecting,
        Indexing,
        Idle,
        Manual
    }

    private IntakeState intakeState = IntakeState.Idle;

    public boolean hasNote() {
        return noteState == NoteState.Indexed || noteState == NoteState.Unindexed;
    }
}
