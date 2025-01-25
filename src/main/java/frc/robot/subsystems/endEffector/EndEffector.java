package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.EndEffector.*;

public class EndEffector extends BlitzSubsystem {
    private final EndEffectorIO io;
    private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
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
        return startEnd(
                () -> io.setSpeed(ENDEFFECTOR_SPEED),
                () -> io.setSpeed(0)
        );
    }
}
