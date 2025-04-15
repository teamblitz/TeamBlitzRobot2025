/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.util.ScoringPositions;
import frc.robot.Constants.AutoConstants.StartingPosition;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ClimbCommandFactory;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.range.RangeSensorIO;
import frc.robot.subsystems.drive.range.RangeSensorIOFusion;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOKraken;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSpark;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOKraken;
import frc.robot.subsystems.superstructure.wrist.WristIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.winch.Winch;
import frc.robot.subsystems.winch.WinchIO;
import frc.robot.subsystems.winch.WinchIOSpark;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */
    private Drive drive;
    private Vision vision;
    private Elevator elevator;
    private Wrist wrist;
    private Intake intake;
    private Superstructure superstructure;
    private Winch winch;
    private Climber climber;
    private AutoCommands autoCommands;

    /* ***** --- Autonomous --- ***** */
    private final AutoChooser autoChooser;

    private final LoggedDashboardChooser<StartingPosition> startingPositionChooser;

    public RobotContainer() {
        CameraServer.startAutomaticCapture();
        configureSubsystems();

        configureButtonBindings();
        setDefaultCommands();
        configureAutoCommands();

        DriverStation.silenceJoystickConnectionWarning(true);
        Shuffleboard.getTab("Drive")
                .add("ResetOdometry", Commands.runOnce(() -> drive.resetOdometry(new Pose2d())));

        //        autoChooser = new LoggedDashboardChooser<>("autoChoice",
        // autoCommands.getFactory().ch);
        autoChooser = new AutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);

        autoCommands = new AutoCommands(drive, superstructure, intake);

        autoChooser.addRoutine("twoPiece", autoCommands::twoPiece);
        //        autoChooser.addRoutine("test", autoCommands::testDrive);
        autoChooser.addRoutine("fourPieceLeft", autoCommands::fourPieceLeft);
        autoChooser.addRoutine("leaveRight", () -> autoCommands.leave("leaveRight"));

        startingPositionChooser = new LoggedDashboardChooser<>("startingPos");
        startingPositionChooser.addDefaultOption("Center", StartingPosition.CENTER);
        startingPositionChooser.addOption("Left", StartingPosition.LEFT);
        startingPositionChooser.addOption("Right", StartingPosition.RIGHT);

        Commands.run(
                        () -> {
                            for (ScoringPositions.Branch branch :
                                    ScoringPositions.Branch.values()) {
                                Logger.recordOutput(
                                        "positions/reef/" + branch.name(),
                                        PositionConstants.Reef.SCORING_POSITIONS.get(branch).get());
                            }
                        })
                .ignoringDisable(true)
                .schedule();
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
                new TeleopSwerve(
                                drive,
                                OIConstants.Drive.X_TRANSLATION,
                                OIConstants.Drive.Y_TRANSLATION,
                                OIConstants.Drive.ROTATION_SPEED,
                                () -> false,
                                () -> Double.NaN,
                                () -> climber.getState() != Climber.State.CLIMB)
                        .unless(RobotState::isTest)
                        .until(RobotState::isTest)
                        .withName("TeleopSwerve"));

        superstructure.setDefaultCommand(
                Commands.either(
                                superstructure
                                        .toGoalThenIdle(Superstructure.Goal.STOW)
                                        .onlyWhile(intake::hasCoral),
                                superstructure
                                        .toGoalThenIdle(Superstructure.Goal.HANDOFF)
                                        .until(intake::hasCoral),
                                intake::hasCoral)
                        .withName("superstructure/conditionalDefault"));
    }

    private void configureSubsystems() {
        drive =
                switch (Constants.ROBOT) {
                    case CompBot ->
                            new Drive(
                                    new SwerveModuleConfiguration(
                                            SwerveModuleConfiguration.MotorType.KRAKEN,
                                            SwerveModuleConfiguration.MotorType.KRAKEN,
                                            SwerveModuleConfiguration.EncoderType.CANCODER),
                                    Constants.Drive.Mod0.CONSTANTS,
                                    Constants.Drive.Mod1.CONSTANTS,
                                    Constants.Drive.Mod2.CONSTANTS,
                                    Constants.Drive.Mod3.CONSTANTS,
                                    new GyroIOPigeon(),
                                    new RangeSensorIOFusion());

                    case DevBot ->
                            new Drive(
                                    new SwerveModuleConfiguration(
                                            SwerveModuleConfiguration.MotorType.NEO,
                                            SwerveModuleConfiguration.MotorType.NEO,
                                            SwerveModuleConfiguration.EncoderType.HELIUM),
                                    Constants.Drive.Mod0.CONSTANTS,
                                    Constants.Drive.Mod1.CONSTANTS,
                                    Constants.Drive.Mod2.CONSTANTS,
                                    Constants.Drive.Mod3.CONSTANTS,
                                    new GyroIOPigeon(),
                                    new RangeSensorIO() {});
                    case SimBot ->
                            new Drive(
                                    new SwerveModule(
                                            Constants.Drive.FL,
                                            new AngleMotorIOSim(),
                                            new DriveMotorIOSim(),
                                            new EncoderIO() {}),
                                    new SwerveModule(
                                            Constants.Drive.FR,
                                            new AngleMotorIOSim(),
                                            new DriveMotorIOSim(),
                                            new EncoderIO() {}),
                                    new SwerveModule(
                                            Constants.Drive.BL,
                                            new AngleMotorIOSim(),
                                            new DriveMotorIOSim(),
                                            new EncoderIO() {}),
                                    new SwerveModule(
                                            Constants.Drive.BR,
                                            new AngleMotorIOSim(),
                                            new DriveMotorIOSim(),
                                            new EncoderIO() {}),
                                    new GyroIOSim(() -> drive.getChassisSpeeds()),
                                    new RangeSensorIO() {});
                };

        vision = new Vision(drive);

        intake = new Intake(Constants.compBot() ? new IntakeIOKraken() : new IntakeIOSpark());

        superstructure =
                new Superstructure(
                        Constants.compBot() ? new ElevatorIO() {} : new ElevatorIOSpark(),
                        Constants.compBot() ? new WristIO() {} : new WristIOSpark());
        elevator = superstructure.getElevator();
        wrist = superstructure.getWrist();

        climber = new Climber(Constants.compBot() ? new ClimberIOKraken() : new ClimberIO() {});
        winch = new Winch(Constants.compBot() ? new WinchIOSpark() : new WinchIO() {});
    }

    private void configureButtonBindings() {
        OIConstants.Drive.RESET_GYRO.onTrue(Commands.runOnce(drive::zeroGyro));
        OIConstants.Drive.X_BREAK.onTrue(drive.park());

        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        OIConstants.SuperStructure.L1.whileTrue(
                superstructure.toGoalThenIdle(Superstructure.Goal.L1));
        OIConstants.SuperStructure.L2.whileTrue(
                superstructure.toGoalThenIdle(Superstructure.Goal.L2));
        OIConstants.SuperStructure.L3.whileTrue(
                superstructure.toGoalThenIdle(Superstructure.Goal.L3));
        OIConstants.SuperStructure.L4.whileTrue(
                superstructure.toGoalThenIdle(Superstructure.Goal.L4));

        OIConstants.SuperStructure.KICK_BOTTOM_ALGAE.whileTrue(
                superstructure
                        .toGoalThenIdle(Superstructure.Goal.KICK_LOW_ALGAE)
                        .alongWith(intake.kick_algae()));
        OIConstants.SuperStructure.KICK_TOP_ALGAE.whileTrue(
                superstructure
                        .toGoalThenIdle(Superstructure.Goal.KICK_HIGH_ALGAE)
                        .alongWith(intake.kick_algae()));

        OIConstants.SuperStructure.HANDOFF.whileTrue(
                CommandFactory.handoff(superstructure, intake));

        OIConstants.SuperStructure.SCORE
                .and(superstructure.triggerAtGoal(Superstructure.Goal.L1))
                .whileTrue(intake.coral_l1());
        OIConstants.SuperStructure.SCORE
                .and(superstructure.triggerAtGoal(Superstructure.Goal.L2))
                .whileTrue(intake.shoot_coral());
        OIConstants.SuperStructure.SCORE
                .and(superstructure.triggerAtGoal(Superstructure.Goal.L3))
                .whileTrue(intake.shoot_coral());
        OIConstants.SuperStructure.SCORE
                .and(superstructure.triggerAtGoal(Superstructure.Goal.L4))
                .onTrue(CommandFactory.l4Plop(superstructure, intake));

        OIConstants.Intake.HANDOFF.whileTrue(intake.handoff());
        OIConstants.Intake.REVERSE.whileTrue(intake.reverse());
        OIConstants.Intake.ALGAE_REMOVAL.whileTrue(intake.kick_algae());
        OIConstants.Intake.SHOOT_CORAL.whileTrue(intake.shoot_coral());

        OIConstants.Intake.INTAKE_ALGAE.whileTrue(intake.setSpeed(Constants.Intake.ALGAE_HOLD));
        OIConstants.Intake.EJECT_ALGAE.whileTrue(intake.setSpeed(Constants.Intake.ALGAE_REMOVAL));

        OIConstants.Winch.WINCH_MAN_UP.whileTrue(winch.manualUp());
        OIConstants.Winch.WINCH_MAN_DOWN.whileTrue(winch.manualDown());

        OIConstants.SuperStructure.MANUAL_MODE.onTrue(
                superstructure.manual(OIConstants.Elevator.MANUAL, OIConstants.Wrist.MANUAL));

        OIConstants.Climber.CLIMBER_UP_MAN.whileTrue(climber.setSpeed(.8));
        OIConstants.Climber.CLIMBER_DOWN_MAN.whileTrue(climber.setSpeed(-.8));

        OIConstants.Climber.DEPLOY_CLIMBER.onTrue(
                ClimbCommandFactory.deployClimber(climber, winch));
        OIConstants.Climber.RESTOW_CLIMBER.onTrue(
                ClimbCommandFactory.stowClimber(climber, winch)
                        .unless(() -> climber.getState() == Climber.State.CLIMB));

        OIConstants.SuperStructure.SCORE
                .and(
                        () ->
                                climber.getState() == Climber.State.DEPLOYED
                                        || climber.getState() == Climber.State.CLIMB)
                .whileTrue(climber.climb());

        new Trigger(RobotController::getUserButton)
                .toggleOnTrue(
                        Commands.parallel(
                                        wrist.coastCommand(),
                                        elevator.coastCommand(),
                                        climber.coastCommand())
                                .onlyWhile(RobotState::isDisabled));

        OIConstants.Drive.ALIGN_LEFT.whileTrue(
                new DeferredCommand(
                        () ->
                                drive.driveToPose(
                                        PositionConstants.Reef.SCORING_POSITIONS.get(
                                                PositionConstants.getClosestFace(drive.getPose())[
                                                        0])),
                        Set.of(drive)));

        OIConstants.Drive.ALIGN_RIGHT.whileTrue(
                new DeferredCommand(
                        () ->
                                drive.driveToPose(
                                        PositionConstants.Reef.SCORING_POSITIONS.get(
                                                PositionConstants.getClosestFace(drive.getPose())[
                                                        1])),
                        Set.of(drive)));

        Commands.run(
                        () -> {
                            PositionConstants.getClosestFace(drive.getPose());
                        })
                .ignoringDisable(true)
                .schedule();
    }

    private void configureAutoCommands() {
        //        NamedCommands.registerCommand(
        //                "score_l3",
        //                superstructure
        //                        .toGoal(Superstructure.Goal.L3)
        //                        .andThen(intake.shoot_coral().withTimeout(1).asProxy()));
        //
        //        NamedCommands.registerCommand(
        //                "score_l4", CommandFactory.l4Plop(superstructure, intake).asProxy());
        //
        //        new
        // EventTrigger("ready_l4").onTrue(superstructure.toGoalThenIdle(Superstructure.Goal.L4));
        //        new EventTrigger("score_l4").onTrue(
        //       Trigger("handoff").onTrue(CommandFactory.handoff(superstructure, intake));
    }

    public Command getAutonomousCommand() {
        Logger.recordOutput("selectedAuto", autoChooser.selectedCommand().getName());
        return Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setGyro(AllianceFlipUtil.shouldFlip() ? 0 : 180)),
                        autoChooser.selectedCommandScheduler())
                .withName("Auto Command");
        //        return Commands.sequence(
        //                        Commands.runOnce(() -> drive.setGyro(180)),
        //                        Commands.parallel(winch.lowerFunnel(),
        // autoChooser.get().asProxy()))
        //                .withName("autonomousCommand");
    }
}
