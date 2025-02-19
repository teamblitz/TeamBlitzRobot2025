package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.math.EqualsUtil;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import org.littletonrobotics.junction.Logger;

public class Elevator extends BlitzSubsystem {
    private final frc.robot.subsystems.superstructure.elevator.ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(0.8, 1.6);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private final SysIdRoutine routine;

    public Elevator(ElevatorIO io) {
        super("elevator");
        this.io = io;

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("Characterization");

        setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
        goal = setpoint;

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(null, Units.Volts.of(5), null),
                        new SysIdRoutine.Mechanism(
                                (volts) -> io.setVolts(volts.in(Units.Volts)), null, this));

        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .withName("Elevator Quasistic Forward"));
        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .withName("Elevator Quasistic Reverse"));

        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kForward).withName("Elevator Dynamic Forward"));
        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("Elevator Dynamic Reverse"));
    }

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
        //
        //        // TODO Set Motor Type and set motor values
        //        if (inputs.topLimitSwitch) {
        //        } else {
        //            left.set(0.0);
        //            right.set(0.0);
        //        }
        //
        //        if (inputs.bottomLimitSwitch) {
        //        } else {
        //            left.set(0.0);
        //            right.set(0.0);
        //        }

        Logger.recordOutput(logKey + "/rotLeftDeg", Math.toDegrees(inputs.positionLeft) % 360);
        Logger.recordOutput(logKey + "/rotRightDeg", Math.toDegrees(inputs.positionRight) % 360);

        // Thou shalt not touch the below code unless it broke
//        setpoint = profile.calculate(Constants.LOOP_PERIOD_SEC, setpoint, goal);
//        TrapezoidProfile.State future_setpoint =
//                profile.calculate(Constants.LOOP_PERIOD_SEC * 2, setpoint, goal);
//
//        io.setSetpoint(
//                setpoint.position,
//                setpoint.velocity,
//                (future_setpoint.velocity - setpoint.velocity) / Constants.LOOP_PERIOD_SEC);
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
                            io.setSpeed(0.6);
                        },
                        () -> {
                            io.setSpeed(0);
                        })
                .alongWith(Commands.print("Up test"));
    }

    public Command downTest() {
        return runEnd(
                        () -> {
                            io.setSpeed(-0.4);
                        },
                        () -> {
                            io.setSpeed(0);
                        })
                .alongWith(Commands.print("dwn test"));
    }

    public Command withGoal(TrapezoidProfile.State goal) {
        return runOnce(
                        () -> {
                            this.goal = goal;
                        })
                .until(() -> setpoint.equals(goal))
                .handleInterrupt(() -> this.goal = setpoint)
                .beforeStarting(refreshCurrentState());
    }

    /**
     * Generates a command to reset the current internal setpoint to the actual state if they
     * conflict
     */
    private Command refreshCurrentState() {
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
                .onlyIf(
                        () ->
                                setpoint == null
                                        || !EqualsUtil.epsilonEquals(
                                                setpoint.position, getPosition(), .02)
                                        || !EqualsUtil.epsilonEquals(
                                                setpoint.velocity, getVelocity(), 0.04));
    }

    private double getPosition() {
        return inputs.positionLeft;
    }

    private double getVelocity() {
        return inputs.velocityLeft;
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrakeMode(false), () -> io.setBrakeMode(true))
                .beforeStarting(() -> Leds.getInstance().armCoast = true)
                .finallyDo(() -> Leds.getInstance().armCoast = false)
                .ignoringDisable(true)
                .withName(logKey + "/coast");
    }
}
