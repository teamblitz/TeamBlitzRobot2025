package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Set;

public class DriveSysId {
    private final CommandSwerveDrivetrain drive;

    @SuppressWarnings("resource")
    public DriveSysId(CommandSwerveDrivetrain drive) {
        this.drive = drive;

        /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
        sysIdRoutineTranslation = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString(
                                "SysIdTranslation_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> drive.setControl(translationCharacterization.withVolts(output)),
                        null,
                        drive));

        /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
        sysIdRoutineSteer = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(7), // Use dynamic voltage of 7 V
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        volts -> drive.setControl(steerCharacterization.withVolts(volts)),
                        null,
                        drive));

        /*
         * SysId routine for characterizing rotation.
         * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
         * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
         */
        sysIdRoutineRotation = new SysIdRoutine(
                new SysIdRoutine.Config(
                        /* This is in radians per second², but SysId only supports "volts per second" */
                        Volts.of(Math.PI / 6).per(Second),
                        /* This is in radians per second, but SysId only supports "volts" */
                        Volts.of(Math.PI),
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> {
                            /* output is actually radians per second, but SysId only supports "volts" */
                            drive.setControl(
                                    rotationCharacterization.withRotationalRate(output.in(Volts)));
                            /* also log the requested output for SysId */
                            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                        },
                        null,
                        drive));

        SendableChooser<SysIdRoutine> routineChooser = new SendableChooser<>();

        routineChooser.addOption("translation", sysIdRoutineTranslation);
        routineChooser.addOption("steer", sysIdRoutineSteer);
        routineChooser.addOption("rotation", sysIdRoutineRotation);

        SendableChooser<SysIdRoutine.Direction> directionChooser = new SendableChooser<>();

        directionChooser.setDefaultOption("forward", SysIdRoutine.Direction.kForward);
        directionChooser.addOption("forward", SysIdRoutine.Direction.kForward);

        SendableChooser<Command> typeChooser = new SendableChooser<>();

        typeChooser.setDefaultOption(
                "quasistatic",
                new DeferredCommand(
                        () -> sysIdQuasistatic(
                                routineChooser.getSelected(), directionChooser.getSelected()),
                        Set.of(drive)));

        typeChooser.addOption(
                "dynamic",
                new DeferredCommand(
                        () -> sysIdDynamic(
                                routineChooser.getSelected(), directionChooser.getSelected()),
                        Set.of(drive)));

        var tab = Shuffleboard.getTab("driveCharacterization");

//        tab.add("sysIdRoutine", routineChooser);
//        tab.add("sysIdDirection", directionChooser);
//        tab.add("sysIdType", typeChooser);
//
//        tab.add("runSysid", typeChooser.getSelected());
    }

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine sysIdRoutineTranslation;

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine sysIdRoutineSteer;

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine sysIdRoutineRotation;

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
