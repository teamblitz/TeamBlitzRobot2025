package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Class to run the rollers over CAN */
public class Roller extends SubsystemBase {
    private final RollerIO io;

    public Roller(RollerIO io) {
        this.io = io;
    }

    // TODO make command better
    public Command runRoller() {
        return startEnd(
                () -> {
                    io.setSpeed(0.3);
                },
                () -> {
                    io.setSpeed(0);
                });
    }
}
