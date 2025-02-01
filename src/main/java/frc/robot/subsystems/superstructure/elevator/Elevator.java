package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;

public class Elevator extends BlitzSubsystem {
    private final frc.robot.subsystems.superstructure.elevator.ElevatorIO io;

    public Elevator(ElevatorIO io) {
        super("elevator");
        this.io = io;
    }

    @Override
    public void periodic() {}

    public Command setSpeed(double left, double right) {
        return startEnd(
                        () -> {
                            io.setSpeedLeft(left);
                            io.setSpeedRight(right);
                        },
                        () -> {
                            io.setSpeedLeft(0);
                            io.setSpeedRight(0);
                        })
                .withName(logKey + "/speed " + left + " " + right);
    }

    public Command upTest() {
        return run(
                () -> {
                    io.setSpeedLeft(0.5);
                    io.setSpeedRight(0.5);
                });
    }

    public Command downTest() {
        return run(
                () -> {
                    io.setSpeedLeft(-0.5);
                    io.setSpeedRight(-0.5);
                });
    }

    // TODO Iplement
    public Command l1Extension() {
        // return Commands.none();
        return runEnd(
                () -> {
                    io.setSpeedLeft(1);
                    io.setSpeedRight(1);
                },
                () -> {
                    io.setMotionMagicLeft(Constants.Elevator.L1_EXTENSION);
                    io.setMotionMagicRight(Constants.Elevator.L1_EXTENSION);
                });
    }

    // TODO: Implement
    public Command l2Extension() {
        return Commands.none();
        //        return runEnd (
        //            () -> {
        //                /* left motor*/(Constants.Elevator.L2_EXTENSION);
        //                /*right motor*/(Constants.Elevator.L2_EXTENSION);
        //            }
        //        )
    }

    // TODO: Implement
    public Command l3Extension() {
        return Commands.none();
        //        return runEnd (
        //            () -> {
        //                /*left motor */(Constants.Elevator.L3_EXTENSION);
        //                /*right motor */(Constants.Elevator.L3_EXTENSION);
        //            }
        //        )
    }

    // TODO: Implement
    public Command l4Extension() {
        return Commands.none();
        //       return runEnd (
        //                /*left motor */(Constants.Elevator.L4_EXTENSION);
        //                /*right motor */(Constants.Elevator.L4_EXTENSION);//          () -> {
        //            }
        //        )
    }

    // TODO Implement
    public Command down() {
        return Commands.none();
    }
}
