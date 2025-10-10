package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Roller.*;

/** Class to run the rollers over CAN */
public class Roller extends SubsystemBase {
    private final RollerIO io;

    public Roller(RollerIO io) {
        super("roller");

        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    // TODO make command better
    public Command runRoller() {
        /*return startEnd(
                () -> {
                    io.setSpeed(0.3);
                },
                () -> {
                    io.setSpeed(0);
                });*/

        return setSpeed(ROLLER_SPEED);
    }

    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));

    }
}
