package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.lib.reefscape.ScoringPositions;
import frc.lib.reefscape.ScoringPositions.Branch;
import frc.robot.Constants;
import frc.robot.PositionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.stream.IntStream;

public class AutoCommands {
    private final CommandSwerveDrivetrain drive;
    private final AutoFactory autoFactory;
    private final Superstructure superstructure;
    private final Intake intake;
    private final Vision vision;

    private final DriveCommands driveCommands;

    //    private final Command configAutonDefault;
    private final Command configTeleDefault;

    private SwerveSample lastSample;

    public AutoCommands(
            CommandSwerveDrivetrain drive,
            Vision vision,
            Superstructure superstructure,
            Intake intake,
            DriveCommands driveCommands) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;
        this.intake = intake;

        this.driveCommands = driveCommands;

        autoFactory = new AutoFactory(
                drive::getPose,
                drive::resetPose,
                sample -> {
                    // Don't ask, just cast (the ring into the fire frodo)
                    lastSample = (SwerveSample) sample;
                    drive.followTrajectory((SwerveSample) sample);
                },
                true,
                drive);

        Command normalDriveDefault = drive.getDefaultCommand();

        // I heard you liked commands, so I gave you a command to set the default command to be a
        // different command
        RobotModeTriggers.autonomous()
                .onTrue(Commands.runOnce(() -> drive.setDefaultCommand(Commands.run(
                        () -> {
                            if (lastSample != null) {
                                Logger.recordOutput("drive/auto/doingDefaultPid", Math.random());
                                drive.followTrajectory(new SwerveSample(
                                        lastSample.t,
                                        lastSample.x,
                                        lastSample.y,
                                        lastSample.heading,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        lastSample.moduleForcesX(),
                                        lastSample.moduleForcesY()));
                            }
                        },
                        drive))));

        // As funny as it would be for the auto to steal the controls of the robot for the rest of
        // the match,
        // unfortunately that is undesired behavior :(, so we need to give them back
        configTeleDefault = Commands.runOnce(() -> drive.setDefaultCommand(normalDriveDefault))
                .ignoringDisable(true);

        RobotModeTriggers.autonomous().onFalse(configTeleDefault);
        RobotModeTriggers.teleop().onTrue(configTeleDefault);
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

        routine.active()
                .whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd())
                        .withName("auto/cmdSec"));

        return routine;
    }

    public AutoRoutine leave(String pathName) {
        final var routine = autoFactory.newRoutine(pathName);
        final var traj = routine.trajectory(pathName);

        routine.active()
                .whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd())
                        .withName("auto/cmdSec"));

        return routine;
    }

    public AutoRoutine twoPiece() {
        AutoRoutine routine = autoFactory.newRoutine("twoPiece");

        AutoTrajectory scoreInitial = routine.trajectory("twoPiece", 0);
        AutoTrajectory toStation = routine.trajectory("twoPiece", 1);
        AutoTrajectory scoreSecond = routine.trajectory("twoPiece", 2);
        AutoTrajectory gtfo = routine.trajectory("twoPiece", 3);

        routine.active()
                .onTrue(Commands.sequence(scoreInitial.resetOdometry(), scoreInitial.cmd()));

        scoreInitial.atTimeBeforeEnd(Constants.Auto.Timings.STOW_TO_L4_READY).onTrue(prepareL4());
        scoreInitial.done().onTrue(scoreL4().andThen(toStation.spawnCmd()));

        toStation.active().onTrue(handoff());

        toStation.chain(scoreSecond);

        scoreSecond
                .atTimeBeforeEnd(Constants.Auto.Timings.STOW_TO_L4_READY)
                .onTrue(Commands.sequence(Commands.waitUntil(intake::hasCoral), prepareL4()));

        scoreSecond
                .done()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(superstructure.triggerAtGoal(Superstructure.Goal.L4)),
                        scoreL4().asProxy(),
                        gtfo.spawnCmd()));

        routine.active().onFalse(configTeleDefault);

        return routine;
    }

    public AutoRoutine fourPieceLeft() {
        return nPiece(4, "fourPieceLeft", List.of(Branch.I, Branch.K, Branch.L, Branch.A));
    }

    /**
     * I channelled my inner AP CSA here, I haven't touched normal for loops in a long time, and it
     * really shows.
     *
     * @param numberOfCoral bingus
     * @param pathName bongus
     * @return boingus
     */
    public AutoRoutine nPiece(
            int numberOfCoral, String pathName, List<ScoringPositions.Branch> scoringPositions) {
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
        List<AutoTrajectory> toReef = IntStream.range(0, (numSplits + 1) / 2)
                .mapToObj(i -> routine.trajectory(pathName, i * 2))
                .toList();
        List<AutoTrajectory> toStation = IntStream.range(0, numSplits / 2)
                .mapToObj(i -> routine.trajectory(pathName, i * 2 + 1))
                .toList();

        routine.active()
                .onTrue(Commands.sequence(
                        toReef.get(0).resetOdometry(), toReef.get(0).cmd()));

        for (int i = 0; i < toReef.size(); i++) {
            toReef.get(i)
                    .atTime(scoringPositions.get(i).name() + "_APPROACH")
                    .onTrue(Commands.parallel(
                            Commands.sequence(
                                    Commands.waitUntil(intake::hasCoral),
                                    // As proxy so that we don't cancel
                                    // handoff if the coral isn't here yet.
                                    prepareL4().asProxy()),
                            drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
                                            scoringPositions.get(i)))
                                    .andThen(
                                            Commands.waitUntil(superstructure.triggerAtGoal(
                                                    Superstructure.Goal.L4)),
                                            scoreL4().asProxy(),
                                            i < toStation.size()
                                                    ? toStation.get(i).spawnCmd()
                                                    : Commands.none())));
        }

        for (int i = 0; i < toStation.size(); i++) {
            toStation.get(i).active().onTrue(handoff());

            // Start the next to reef section if it exists
            if (i + 1 < toReef.size()) {
                toStation.get(i).chain(toReef.get(i + 1));
            }
        }

        routine.active().onFalse(configTeleDefault);

        return routine;
    }

    private Command scoreL4() {
        return CommandFactory.l4Plop(superstructure, intake);
    }

    private Command prepareL4() {
        return superstructure.toGoalThenIdle(Superstructure.Goal.L4);
    }

    private Command handoff() {
        return CommandFactory.handoff(superstructure, intake);
    }

    public AutoRoutine demoFollowAprilTag() {
        AutoRoutine routine = autoFactory.newRoutine("demoFollowAprilTag");

        var pose = new Pose2d(-1, 0, Rotation2d.kZero);

        routine.active()
                .whileTrue(runOnce(() -> {
                            drive.resetPose(pose);
                            vision.setAprilTagLayout(Vision.DEMO_FOLLOW_LAYOUT);
                        })
                        .andThen(waitSeconds(1))
                        .andThen(driveCommands.pullToPose(
                                () -> pose, 4, 1, 2, .05, .5 * Math.PI, Math.PI))
                        .finallyDo(() -> vision.setAprilTagLayout(Vision.DEFAULT_FIELD_LAYOUT)));

        return routine;
    }
}
