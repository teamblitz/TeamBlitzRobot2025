/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.MutableReference;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.AutoConstants.StartingPosition;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSpark;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONavx;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.drive.range.RangeSensorIO;
import frc.robot.subsystems.drive.range.RangeSensorIOFusion;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.notes.NoteVisionIO;
import frc.robot.subsystems.vision.notes.NoteVisionIOLimelight;
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
    private Intake intake;
    private Shooter shooter;
    private Arm arm;
    private Climber climber;

    /* ***** --- Shared Commands --- ***** */
    public Command autoShootSpeed;

    /* ***** --- Autonomous --- ***** */
    private final LoggedDashboardChooser<Command> autoChooser;

    private final LoggedDashboardChooser<StartingPosition> startingPositionChooser;

    public RobotContainer() {
        Leds.getInstance();

        configureSubsystems();

        configureButtonBindings();
        setDefaultCommands();
        configureAutoCommands();

        DriverStation.silenceJoystickConnectionWarning(true);
        Shuffleboard.getTab("Drive")
                .add("ResetOdometry", Commands.runOnce(() -> drive.resetOdometry(new Pose2d())));

        autoChooser = new LoggedDashboardChooser<>("autoChoice", AutoBuilder.buildAutoChooser());

        startingPositionChooser = new LoggedDashboardChooser<>("startingPos");
        startingPositionChooser.addDefaultOption("Center", StartingPosition.CENTER);
        startingPositionChooser.addOption("Left", StartingPosition.LEFT);
        startingPositionChooser.addOption("Right", StartingPosition.RIGHT);

        //        Shuffleboard.getTab("AutoShoot")
        //                .addDouble(
        //                        "distance",
        //                        () ->
        //                                AutoAimCalculator.calculateDistanceToGoal(
        //                                        new Pose3d(drive.getEstimatedPose())));
        //
        //        Shuffleboard.getTab("AutoShoot")
        //                .addDouble(
        //                        "speed",
        //                        () ->
        //                                AutoAimCalculator.calculateShooterSpeedInterpolation(
        //                                        AutoAimCalculator.calculateDistanceToGoal(
        //                                                new Pose3d(drive.getEstimatedPose()))));
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
                new TeleopSwerve(
                                drive,
                                OIConstants.Drive.X_TRANSLATION,
                                OIConstants.Drive.Y_TRANSLATION,
                                OIConstants.Drive.ROTATION_SPEED,
                                () -> false,
                                () -> // TODO: THis should be declarative, when button do xyz, not
                                        // imperative as it is currently
                                        OIConstants.Drive.AMP_ASSIST.getAsBoolean()
                                                ? (DriverStation.getAlliance().isPresent()
                                                                && DriverStation.getAlliance().get()
                                                                        == DriverStation.Alliance
                                                                                .Red
                                                        ? 90
                                                        : -90)
                                                : Double.NaN)
                        .withName("TeleopSwerve"));

        new Trigger(() -> Math.abs(OIConstants.Arm.MANUAL_ARM_SPEED.getAsDouble()) > .08)
                .whileTrue(
                        Commands.run(
                                        () -> {
                                            arm.setArmRotationSpeed(
                                                    OIConstants.Arm.MANUAL_ARM_SPEED.getAsDouble());
                                        })
                                .finallyDo(() -> arm.setArmRotationSpeed(0))
                                .raceWith(arm.setGoal(Arm.Goals.MANUAL))
                                .withName("arm/manual"));
    }

    private void configureSubsystems() {
        drive =
                switch (Constants.ROBOT) {
                    case CompBot -> new Drive(
                            new SwerveModuleConfiguration(
                                    SwerveModuleConfiguration.MotorType.KRAKEN,
                                    SwerveModuleConfiguration.MotorType.NEO,
                                    SwerveModuleConfiguration.EncoderType.CANCODER),
                            Constants.Drive.Mod0.CONSTANTS,
                            Constants.Drive.Mod1.CONSTANTS,
                            Constants.Drive.Mod2.CONSTANTS,
                            Constants.Drive.Mod3.CONSTANTS,
                            Constants.Drive.USE_PIGEON ? new GyroIOPigeon() : new GyroIONavx(),
                            new RangeSensorIOFusion(),
                            new NoteVisionIOLimelight("limelight-intake"));

                    case DevBot -> new Drive(
                            new SwerveModuleConfiguration(
                                    SwerveModuleConfiguration.MotorType.NEO,
                                    SwerveModuleConfiguration.MotorType.NEO,
                                    SwerveModuleConfiguration.EncoderType.HELIUM),
                            Constants.Drive.Mod0.CONSTANTS,
                            Constants.Drive.Mod1.CONSTANTS,
                            Constants.Drive.Mod2.CONSTANTS,
                            Constants.Drive.Mod3.CONSTANTS,
                            Constants.Drive.USE_PIGEON ? new GyroIOPigeon() : new GyroIONavx(),
                            new RangeSensorIO() {},
                            new NoteVisionIO() {});
                    case SimBot -> new Drive(
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
                            new GyroIO() {},
                            new RangeSensorIO() {},
                            new NoteVisionIO() {});
                };

        intake = new Intake(new IntakeIOSpark(), OIConstants.Overrides.INTAKE_OVERRIDE);
        shooter = new Shooter(new ShooterIOSpark());
        arm = new Arm(new ArmIOSpark(true));

        arm.setAimGoal(
                () ->
                        AutoAimCalculator.calculateArmAngleInterpolation(
                                AutoAimCalculator.calculateDistanceToGoal(
                                        new Pose3d(drive.getEstimatedPose()))));

        climber = new Climber(Constants.compBot() ? new ClimberIOKraken() {} : new ClimberIO() {});

        autoShootSpeed =
                shooter.shootClosedLoopCommand(
                        () ->
                                AutoAimCalculator.calculateShooterSpeedInterpolation(
                                        AutoAimCalculator.calculateDistanceToGoal(
                                                new Pose3d(drive.getLimelightPose()))));
    }

    private void configureButtonBindings() {
        OIConstants.Drive.RESET_GYRO.onTrue(Commands.runOnce(drive::zeroGyro));
        OIConstants.Drive.X_BREAK.onTrue(drive.park());

        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        NetworkTableEntry intakeTx =
                LimelightHelpers.getLimelightNTTableEntry("limelight-intake", "tx");
        NetworkTableEntry intakeTv =
                LimelightHelpers.getLimelightNTTableEntry("limelight-intake", "tv");

        Debouncer tvBouncer = new Debouncer(2. / 30., Debouncer.DebounceType.kBoth);
        MutableReference<Double> txCache = new MutableReference<>(0.);
        MutableReference<Boolean> tvCache = new MutableReference<>(false);

        OIConstants.UNBOUND.whileTrue(
                drive.chaseVector(
                                () ->
                                        new Translation2d(
                                                        Math.cos(
                                                                Math.toRadians(
                                                                        txCache.get() * 1.7)),
                                                        Math.sin(
                                                                Math.toRadians(
                                                                        txCache.get() * 1.7)))
                                                .rotateBy(drive.getYaw()),
                                () -> -txCache.get(),
                                2,
                                4)
                        .until(() -> !tvCache.get())
                        .beforeStarting(() -> Leds.getInstance().autoPickupActive = true)
                        .finallyDo(() -> Leds.getInstance().autoPickupActive = false)
                        .onlyIf(tvCache::get));

        Commands.run(
                        () -> {
                            tvCache.set(tvBouncer.calculate(intakeTv.getDouble(0) == 1));
                            if (intakeTv.getDouble(0) == 1) {
                                txCache.set(intakeTx.getDouble(0));
                            }
                        })
                .ignoringDisable(true)
                .schedule();

        new Trigger(() -> tvBouncer.calculate(intakeTv.getDouble(0) == 1))
                .whileTrue(
                        Commands.startEnd(
                                        () -> Leds.getInstance().autoPickupReady = true,
                                        () -> Leds.getInstance().autoPickupReady = false)
                                .ignoringDisable(true));

        OIConstants.Drive.AMP_ASSIST.whileTrue(
                drive.useVelocityFilter(drive.ampAssistFilter)
                        .alongWith(arm.setGoal(Arm.Goals.AMP_BACK)));
        OIConstants.Drive.AUTO_PICKUP.whileTrue(drive.useVelocityFilter(drive.noteAssistFilter));

        OIConstants.Intake.FEED.whileTrue(intake.feedShooter());
        OIConstants.Intake.EJECT.whileTrue(intake.ejectCommand());
        OIConstants.Shooter.MANUAL_FEED.whileTrue(shooter.shootCommand());
        OIConstants.Shooter.SHOOTER_AMP.whileTrue(shooter.shootCommand());
        OIConstants.Shooter.EJECT.whileTrue(shooter.reverseCommand());
        OIConstants.Shooter.SPEED_AUTO.whileTrue(autoShootSpeed);

        OIConstants.Intake.SCORE
                .and(OIConstants.Arm.AMP_BACK)
                .whileTrue(ManipulatorCommands.scoreAmpBack(shooter, intake));

        OIConstants.Intake.SCORE
                .and(OIConstants.Arm.AMP_FRONT)
                .whileTrue(ManipulatorCommands.scoreAmpFront(shooter, intake));

        OIConstants.Arm.INTAKE.whileTrue(ManipulatorCommands.intakeGround(intake, arm));

        OIConstants.Arm.SPEAKER_SUB_FRONT.whileTrue(
                ManipulatorCommands.shootSubwoofer(shooter, arm));
        OIConstants.Arm.SPEAKER_PODIUM.whileTrue(ManipulatorCommands.shootPodium(shooter, arm));

        OIConstants.Arm.AMP_BACK.whileTrue(arm.setGoal(Arm.Goals.AMP_BACK));
        OIConstants.Arm.AMP_FRONT.whileTrue(arm.setGoal(Arm.Goals.AMP_FRONT));

        OIConstants.Arm.AUTO_AIM_SPEAKER.whileTrue(
                ManipulatorCommands.shootAim(shooter, arm, drive::getEstimatedPose));

        // CLIMBER COMMANDS
        OIConstants.Climber.UP_BOTH.whileTrue(
                climber.goUp()
                        .beforeStarting(() -> Leds.getInstance().climbing = true)
                        .finallyDo(() -> Leds.getInstance().climbing = false));
        OIConstants.Climber.DOWN_BOTH.whileTrue(
                climber.climb()
                        .beforeStarting(() -> Leds.getInstance().climbing = true)
                        .finallyDo(() -> Leds.getInstance().climbing = false));
        OIConstants.Climber.DOWN_MANUAL.whileTrue(
                climber.setSpeed(-0.3, -0.3)
                        .beforeStarting(() -> Leds.getInstance().climbing = true)
                        .finallyDo(() -> Leds.getInstance().climbing = false));

        // Lower the arm when climbing
        OIConstants.Climber.DOWN_BOTH.onTrue(arm.setGoal(Arm.Goals.CLIMB));
        OIConstants.Climber.UP_BOTH.onTrue(arm.setGoal(Arm.Goals.CLIMB));

        // Stage avoidance
        arm.setStageSafety(OIConstants.Arm.TRANSIT_STAGE);

        // TEST STUFF
        //        OIConstants.TestMode.ZERO_ABS_ENCODERS.onTrue(drive.zeroAbsEncoders());
        //        OIConstants.TestMode.SysId.Arm.QUASISTATIC_FWD.whileTrue(
        //                arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        //                        .beforeStarting(Commands.print("ArmQuasFwd")));
        //        OIConstants.TestMode.SysId.Arm.QUASISTATIC_REV.whileTrue(
        //                arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        //                        .beforeStarting(Commands.print("ArmQuasRev")));
        //        OIConstants.TestMode.SysId.Arm.DYNAMIC_FWD.whileTrue(
        //                arm.sysIdDynamic(SysIdRoutine.Direction.kForward)
        //                        .beforeStarting(Commands.print("ArmDynamicFwd")));
        //        OIConstants.TestMode.SysId.Arm.DYNAMIC_REV.whileTrue(
        //                arm.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        //                        .beforeStarting(Commands.print("ArmDynamicRev")));
        //
        //        OIConstants.TestMode.SysId.Drive.QUASISTATIC_FWD.whileTrue(
        //                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        //                        .beforeStarting(Commands.print("DriveQuasFwd")));
        //        OIConstants.TestMode.SysId.Drive.QUASISTATIC_REV.whileTrue(
        //                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        //                        .beforeStarting(Commands.print("DriveQuasRev")));
        //        OIConstants.TestMode.SysId.Drive.DYNAMIC_FWD.whileTrue(
        //                drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        //                        .beforeStarting(Commands.print("DriveDynamicFwd")));
        //        OIConstants.TestMode.SysId.Drive.DYNAMIC_REV.whileTrue(
        //                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        //                        .beforeStarting(Commands.print("DriveDynamicRev")));

        new Trigger(RobotController::getUserButton).toggleOnTrue(arm.coastCommand());
    }

    private void configureAutoCommands() {

        // This is the worst, no need to mess with it too much until we get choreo set up, but
        // seriously we need to fix this.
        InternalButton readyShoot = new InternalButton();
        InternalButton shoot = new InternalButton();
        InternalButton qShoot = new InternalButton();

        //        shoot.whileTrue(
        //                arm.setGoal(Arm.Goals.SUBWOOFER)
        //                        .raceWith(Commands.waitSeconds(1))
        //                        .andThen(intake.feedShooter().asProxy().withTimeout(.5))
        //                        .raceWith(shooter.shootCommand())
        //                        .asProxy()
        //                        .withName("auto/shoot"));

        shoot.whileTrue(
                arm.setGoal(Arm.Goals.SUBWOOFER)
                        .raceWith(
                                Commands.waitSeconds(1)
                                        .andThen(intake.feedShooter().asProxy().withTimeout(.5))
                                        .raceWith(shooter.shootCommand())
                                        .asProxy())
                        .withName("auto/shoot"));

        qShoot.whileTrue(
                arm.setGoal(Arm.Goals.SUBWOOFER)
                        .alongWith(intake.feedShooter(.7).asProxy())
                        .raceWith(shooter.shootCommand())
                        .asProxy()
                        .withTimeout(.75)
                        .withName("auto/qShoot"));

        readyShoot.whileTrue(
                arm.setGoal(Arm.Goals.SUBWOOFER)
                        .asProxy()
                        .alongWith(shooter.shootCommand().asProxy())
                        .asProxy()
                        .withName("auto/readyShoot"));
        // Does end
        NamedCommands.registerCommand(
                "shoot",
                Commands.runOnce(
                                () -> {
                                    shoot.setPressed(true);
                                    qShoot.setPressed(false);
                                    readyShoot.setPressed(false);
                                })
                        .andThen(Commands.waitSeconds(1.5)));

        NamedCommands.registerCommand(
                "qshoot",
                Commands.runOnce(
                                () -> {
                                    shoot.setPressed(false);
                                    qShoot.setPressed(true);
                                    readyShoot.setPressed(false);
                                })
                        .andThen(Commands.waitSeconds(.75)));

        // Does not end
        NamedCommands.registerCommand(
                "intake",
                arm.setGoal(Arm.Goals.INTAKE)
                        .asProxy()
                        .alongWith(intake.intakeGroundAutomatic(.7).asProxy())
                        .alongWith(shooter.setSpeedCommand(-.1).asProxy()));

        NamedCommands.registerCommand(
                "readyShoot",
                Commands.runOnce(
                                () -> {
                                    System.out.println("READY SHOOT");
                                    shoot.setPressed(false);
                                    qShoot.setPressed(false);
                                    readyShoot.setPressed(true);
                                })
                        .andThen(Commands.waitSeconds(1)));
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        return Commands.runOnce(() -> drive.setGyro(startingPositionChooser.get().angle))
                .andThen(autoChooser.get().asProxy());
    }
}
