package frc.robot.subsystems.superstructure.winch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.intake.IntakeInputsAutoLogged;
import static frc.robot.Constants.Winch.*;
import org.littletonrobotics.junction.Logger;

public class Winch extends BlitzSubsystem{
    private final WinchIO io;
    //private final WinchInputsAutoLogged inputs = new WinchInputsAutoLogged();

    public Winch () {
        super("winch");
        this.io = io;
    }
    @Override
    public void periodic() {
        super.periodic();

        //io.updateInputs(inputs);
        //Logger.processInputs(logKey, inputs);
    }
    public Command funnal_Up () {
        return startEnd(
                () -> io.setSpeed(FUNNAL_UP),
                () -> io.setSpeed(0));
    }
    public Command funnal_Down () {
        return startEnd(
                () -> io.setSpeed(FUNNAL_DOWN),
                () -> io.setSpeed(0));
    }

}
