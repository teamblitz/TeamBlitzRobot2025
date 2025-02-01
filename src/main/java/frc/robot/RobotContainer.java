/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.MutableReference;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.AutoConstants.StartingPosition;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.drive.range.RangeSensorIO;
import frc.robot.subsystems.drive.range.RangeSensorIOFusion;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIO;
import frc.robot.subsystems.leds.Leds;
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
                                        //
                                        // OIConstants.Drive.AMP_ASSIST.getAsBoolean()
                                        //                                                ?
                                        // (DriverStation.getAlliance().isPresent()
                                        //
                                        //      && DriverStation.getAlliance().get()
                                        //
                                        //              == DriverStation.Alliance
                                        //
                                        //                      .Red
                                        //                                                        ?
                                        // 90
                                        //                                                        :
                                        // -90)
                                        Double.NaN)
                        .withName("TeleopSwerve"));
    }

    private void configureSubsystems() {
        drive =
                switch (Constants.ROBOT) {
                    case CompBot ->
                            new Drive(
                                    new SwerveModuleConfiguration(
                                            SwerveModuleConfiguration.MotorType.KRAKEN,
                                            SwerveModuleConfiguration.MotorType.NEO,
                                            SwerveModuleConfiguration.EncoderType.CANCODER),
                                    Constants.Drive.Mod0.CONSTANTS,
                                    Constants.Drive.Mod1.CONSTANTS,
                                    Constants.Drive.Mod2.CONSTANTS,
                                    Constants.Drive.Mod3.CONSTANTS,
                                    new GyroIOPigeon(),
                                    new RangeSensorIOFusion(),
                                    new NoteVisionIOLimelight("limelight-intake"));

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
                                    new RangeSensorIO() {},
                                    new NoteVisionIO() {});
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
                                    new GyroIO() {},
                                    new RangeSensorIO() {},
                                    new NoteVisionIO() {});
                };
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

        new Trigger(() -> )
    }

    private void configureAutoCommands() {}

    public Command getAutonomousCommand() { // Autonomous code goes here
        return Commands.runOnce(() -> drive.setGyro(startingPositionChooser.get().angle))
                .andThen(autoChooser.get().asProxy());
    }
}
