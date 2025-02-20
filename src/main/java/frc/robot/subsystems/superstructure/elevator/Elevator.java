package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.math.EqualsUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends BlitzSubsystem {
    private final frc.robot.subsystems.superstructure.elevator.ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(0.2, 0.4);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private final SysIdRoutine routine;

    // Left Elevator Tunable Numbers
    private final LoggedTunableNumber leftKP =
            new LoggedTunableNumber("elevator/kP", Constants.Elevator.Gains.KP);
    private final LoggedTunableNumber leftKI =
            new LoggedTunableNumber("elevator/kI", Constants.Elevator.Gains.KI);
    private final LoggedTunableNumber leftKD =
            new LoggedTunableNumber("elevator/kD", Constants.Elevator.Gains.KD);


    private final LoggedTunableNumber leftKS =
            new LoggedTunableNumber("elevator/kS", Constants.Elevator.Gains.KS);
    private final LoggedTunableNumber leftKV =
            new LoggedTunableNumber("elevator/kV", Constants.Elevator.Gains.KV);
    private final LoggedTunableNumber leftKA =
            new LoggedTunableNumber("elevator/kA", Constants.Elevator.Gains.KA);
    private final LoggedTunableNumber leftKG =
            new LoggedTunableNumber("elevator/kG", Constants.Elevator.Gains.KG);

    public Elevator(ElevatorIO io) {
        super("elevator");
        this.io = io;

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("Characterization");

        setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
        goal = null;


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

        characterizationTab.add(
                "elevator/0.2m",
                withGoal(new TrapezoidProfile.State(.2,0)).withName("elevator/0.2m test"));

        characterizationTab.add(
                "elevator/0.4m",
                withGoal(new TrapezoidProfile.State(.4,0)).withName("elevator/0.4m test"));

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


        if (goal != null) {
            setpoint = profile.calculate(Constants.LOOP_PERIOD_SEC, setpoint, goal);
            TrapezoidProfile.State future_setpoint =
                    profile.calculate(Constants.LOOP_PERIOD_SEC * 2, setpoint, goal);

            io.setSetpoint(
                    setpoint.position,
                    setpoint.velocity,
                    future_setpoint.velocity);

            Logger.recordOutput(logKey + "/profile/positionSetpoint", setpoint.position);
            Logger.recordOutput(logKey + "/profile/velocitySetpoint", setpoint.velocity);

            Logger.recordOutput(logKey + "/profile/positionGoal", goal.position);
            Logger.recordOutput(logKey + "/profile/velocityGoal", goal.velocity);
        }

        if (DriverStation.isDisabled()) {
            // Reset profile when disabled
            setpoint = new TrapezoidProfile.State(getPosition(), 0);
            goal = null;

            // Stop arm
            io.stop();
            return;
        }


        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        LoggedTunableNumber.ifChanged(
                hashCode(), pid -> io.setPid(pid[0], pid[1], pid[2]), leftKP, leftKI, leftKD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                (kSGVA) -> io.setFF(kSGVA[0], kSGVA[1], kSGVA[2], kSGVA[3]),
                leftKS,
                leftKG,
                leftKV,
                leftKA);

//        LoggedTunableNumber.ifChanged(
//                hashCode(), pid -> io.setPid(pid[0], pid[1], pid[2]), rightKP, rightKD);
//
//        LoggedTunableNumber.ifChanged(
//                hashCode(),
//                kSGVA -> io.setFF()  = new ElevatorFeedforward(kSGVA[0], kSGVA[1], kSGVA[2], kSGVA[3]),
//                rightKS,
//                rightKV,
//                rightKA,
//                rightKG);
    }

    public Command setSpeed(double left, double right) {
        return startEnd(
                        () -> {
                            io.setSpeed(left);
                        },
                        () -> {
                            io.setSpeed(0);
                        })
                .withName(logKey + "/speed " + left + " " + right)
                .beforeStarting(
                        () -> this.goal = null
                );
    }

    public Command upTest() {
        return runEnd(
                        () -> {
                            io.setSpeed(0.2);
                        },
                        () -> {
                            io.setSpeed(0);
                        })
                .alongWith(Commands.print("Up test"));
    }

    public Command downTest() {
        return runEnd(
                        () -> {
                            io.setSpeed(-0.2);
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

    @AutoLogOutput(key = "elevator/position")
    private double getPosition() {
        return inputs.positionLeft;
    }

    @AutoLogOutput(key = "elevator/velocity")
    private double getVelocity() {
        return inputs.velocityLeft;
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
