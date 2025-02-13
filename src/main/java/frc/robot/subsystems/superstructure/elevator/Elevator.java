package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends BlitzSubsystem {
    private final frc.robot.subsystems.superstructure.elevator.ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.8, 1.6);
    private final TrapezoidProfile profile
            = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public Elevator(ElevatorIO io) {
        super("elevator");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(logKey + "/rotLeftDeg", Math.toDegrees(inputs.positionLeft) % 360);
        Logger.recordOutput(logKey + "/rotRightDeg", Math.toDegrees(inputs.positionRight) % 360);

        // Thou shalt not touch the below code unless it broke
//        setpoint = profile.calculate(Constants.LOOP_PERIOD_SEC, setpoint, goal);
//        TrapezoidProfile.State future_setpoint = profile.calculate(Constants.LOOP_PERIOD_SEC * 2, setpoint, goal);
//
//        io.setSetpoint(setpoint.position, setpoint.velocity, (future_setpoint.velocity - setpoint.velocity) / Constants.LOOP_PERIOD_SEC);
    }

    public Command setSpeed(double left, double right) {
        return startEnd(
                        () -> {
                            io.setSpeed(left);
                        },
                        () -> {
                            io.setSpeed(0);
                        })
                .withName(logKey + "/speed " + left + " " + right);
    }

    public Command upTest() {
        return runEnd(
                () -> {
                    io.setSpeed(0.2);
                },
                () -> {
                    io.setSpeed(0);
                }).alongWith(Commands.print("Up test"));
    }

    public Command downTest() {
        return runEnd(
                () -> {
                    io.setSpeed(-0.2);
                },
                () -> {
                    io.setSpeed(0);
                }).alongWith(Commands.print("dwn test"));
    }



    // TODO Implement
    public Command l1Extension() {
        // return Commands.none();
        return runEnd(
                () -> {
                    io.setSpeed(1);
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
