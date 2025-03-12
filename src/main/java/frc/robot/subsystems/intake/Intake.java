package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends BlitzSubsystem {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake(IntakeIO io) {
        super("intake");

        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    public Command handoff() {
        return startEnd(() -> io.setSpeed(HANDOFF_SPEED), () -> io.setSpeed(0));
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
}
