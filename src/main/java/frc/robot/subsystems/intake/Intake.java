package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.INTAKE_SPEED;
import static frc.robot.Constants.Intake.OUTTAKE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends BlitzSubsystem {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake(IntakeIO io) {
        super("Intake");

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

    public Command outtake() {
        return startEnd(() -> io.setSpeed(OUTTAKE_SPEED), () -> io.setSpeed(0));
    }
}
