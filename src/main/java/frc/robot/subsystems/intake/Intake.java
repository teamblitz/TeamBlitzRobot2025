package frc.robot.subsystems.intake;

import static frc.robot.Constants.EndEffector.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends BlitzSubsystem {
    private final IntakeIO io;
    private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();

    public Intake(IntakeIO io) {
        super("EndEffector");

        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    public Command intake() {
        return startEnd(() -> io.setSpeed(INTAKE_SPEED), () -> io.setSpeed(0));
    }

    // TODO: Implement
    public Command outtake() {
        throw new UnsupportedOperationException();
    }
}
