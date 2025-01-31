package frc.robot.subsystems.intake;

import static frc.robot.Constants.EndEffector.OUTTAKE_SPEED;
import static frc.robot.Constants.EndEffector.INTAKE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.intake.IntakeInputsAutoLogged; //TODO import the new autolog & fix other autologs
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.Constants;

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

    public Command outtake() {
        return startEnd(
            () -> io.setSpeed(OUTTAKE_SPEED),
            () -> io.setSpeed(0));
    }

}
