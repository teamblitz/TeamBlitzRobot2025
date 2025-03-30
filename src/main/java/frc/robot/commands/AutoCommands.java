package frc.robot.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

import java.util.List;
import java.util.function.Consumer;
import java.util.stream.IntStream;


public class AutoCommands {
    private final Drive drive;
    private final SwerveDriveKinematics kinematics;
    private final AutoFactory autoFactory;
    private final Superstructure superstructure;
    private final Intake intake;

    private SwerveSample lastSample;

    public AutoCommands(Drive drive, Superstructure superstructure, Intake intake) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.intake = intake;
        this.kinematics = Constants.Drive.KINEMATICS;


        autoFactory = new AutoFactory(
            drive::getPose,
            drive::resetOdometry,
            sample -> {
                // Don't ask, just cast (the ring into the fire frodo)
                lastSample = (SwerveSample) sample;
                drive.followTrajectory((SwerveSample) sample);
            },
            true,
            drive
        );

        Command normalDriveDefault = drive.getDefaultCommand();

        // I heard you liked commands, so I gave you a command to set the default command to be a different command
        RobotModeTriggers.autonomous().onTrue(
                Commands.runOnce(
                        () -> drive.setDefaultCommand(
                                Commands.run(
                                        () -> drive.followTrajectory(lastSample)
                                )
                        )
                )
        );

        // As funny as it would be for the auto to steal the controls of the robot for the rest of the match,
        // unfortunately that is undesired behavior :(, so we need to give them back
        RobotModeTriggers.autonomous().onFalse(
                Commands.runOnce(
                        () -> drive.setDefaultCommand(normalDriveDefault)
                )
        );

    }

    public AutoFactory getFactory() {
        return autoFactory;
    }

    public Command getNoAuto() {
        final var routine = autoFactory.newRoutine("None");
        routine.active().onTrue(Commands.print("Running No Auto"));


        return routine.cmd();
    }

    public AutoRoutine testDrive() {
        final var routine = autoFactory.newRoutine("test");
        final var traj = routine.trajectory("test");

        routine.active().whileTrue(
                Commands.sequence(
                        traj.resetOdometry(),
                        traj.cmd()
                ).withName("auto/cmdSec")
        );


        return routine;
    }

    public AutoRoutine twoPiece() {
        AutoRoutine routine = autoFactory.newRoutine("twoPiece");

        AutoTrajectory scoreInitial = routine.trajectory("twoPiece", 0);
        AutoTrajectory toStation = routine.trajectory("twoPiece",1);
        AutoTrajectory scoreSecond = routine.trajectory("twoPiece", 2);
        AutoTrajectory gtfo = routine.trajectory("twoPiece", 3);


        routine.active().onTrue(
                Commands.sequence(
                        scoreInitial.resetOdometry(),
                        scoreInitial.cmd()
                )
        );



        scoreInitial.atTimeBeforeEnd(Constants.Auto.Timings.STOW_TO_L4_READY).onTrue(prepareL4());
        scoreInitial.done().onTrue(
                scoreL4().andThen(toStation.spawnCmd()));


        toStation.active().onTrue(handoff());

        toStation.chain(scoreSecond);


        scoreSecond.atTimeBeforeEnd(Constants.Auto.Timings.STOW_TO_L4_READY)
                        .onTrue(
                                Commands.sequence(
                                        Commands.waitUntil(intake::hasCoral),
                                        prepareL4()
                                ));

        scoreSecond.done().onTrue(
                Commands.sequence(
                        Commands.waitUntil(superstructure.triggerAtGoal(Superstructure.Goal.L4)),
                        scoreL4(),
                        gtfo.spawnCmd()
                )
        );



        return routine;

    }


    public AutoRoutine fourPieceLeft() {
        return nPiece(4, "fourPieceLeft");
    }

    /**
     * I channelled my inner AP CSA here, I haven't touched normal for loops in a long time, and it really shows.
     *
     * @param numberOfCoral bingus
     * @param pathName bongus
     * @return boingus
     */
    public AutoRoutine nPiece(int numberOfCoral, String pathName) {
        if (!List.of(1, 2, 3, 4).contains(numberOfCoral)) {
            throw new IllegalArgumentException("Number of Coral must be between 1 and 4");
        }

        AutoRoutine routine = autoFactory.newRoutine(pathName);


        var trajectory = Choreo.loadTrajectory(pathName);
        if (trajectory.isEmpty()) {
            DriverStation.reportError("No trajectory found for " + pathName, false);
            return routine;
        }


        int numSplits = trajectory.get().splits().size();

        // Even splits are toReef trajectories
        // Odd splits are to station trajectories
        List<AutoTrajectory> toReef = IntStream.range(0, (numSplits + 1) / 2).mapToObj(i -> routine.trajectory(pathName, i * 2)).toList();
        List<AutoTrajectory> toStation = IntStream.range(0, numSplits / 2).mapToObj(i -> routine.trajectory(pathName, i * 2 + 1)).toList();


        routine.active().onTrue(
                Commands.sequence(
                        toReef.get(0).resetOdometry(),
                        toReef.get(0).cmd()
                )
        );

        for (int i = 0; i < toReef.size(); i++) {
            toReef.get(i).atTimeBeforeEnd(Constants.Auto.Timings.STOW_TO_L4_READY).onTrue(
                    Commands.sequence(
                            Commands.waitUntil(intake::hasCoral),
                            prepareL4())
            );
            toReef.get(i).done().onTrue(
                    scoreL4().andThen(
                            i < toStation.size() ?
                            toStation.get(i).spawnCmd() : Commands.none())
            );
        }

        for (int i = 0; i < toStation.size(); i++) {
            toStation.get(i).active().onTrue(handoff());

            // Start the next to reef section if it exists
            if (i + 1 < toReef.size()) {
                toStation.get(i).chain(toReef.get(i + 1));
            }
        }

        return routine;
    }

    private Command scoreL4() {
        return Commands.sequence(
                superstructure.toGoal(Superstructure.Goal.L4),
                CommandFactory.l4Plop(superstructure, intake)
        );
    }

    private Command prepareL4() {
        return superstructure.toGoalThenIdle(Superstructure.Goal.L4);
    }

    private Command handoff() {
        return CommandFactory.handoff(superstructure, intake);
    }

}
