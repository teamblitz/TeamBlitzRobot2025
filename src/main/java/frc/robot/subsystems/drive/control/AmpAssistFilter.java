package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public class AmpAssistFilter extends ChassisSpeedFilter {
    private final TrapezoidProfile motionProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

    private static final LoggedTunableNumber p = new LoggedTunableNumber("drive/amp-assist/kP", 1);
    private static final LoggedTunableNumber d = new LoggedTunableNumber("drive/amp-assist/kD", 0);

    private ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(p.get(), 0, d.get(), new TrapezoidProfile.Constraints(1, 2));

    private TrapezoidProfile.State state = new TrapezoidProfile.State();
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State(.3, 0);

    public AmpAssistFilter(Drive drive) {
        super(drive, true);

        profiledPIDController.setGoal(goal);
        Shuffleboard.getTab("AHHHH").add("AMP ASSIST", profiledPIDController);
    }

    @Override
    public ChassisSpeeds apply(ChassisSpeeds initialSpeeds) {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                (pd) -> {
                    profiledPIDController.setPID(pd[0], 0, pd[1]);
                },
                p,
                d);

        state =
                motionProfile.calculate(
                        Robot.defaultPeriodSecs,
                        new TrapezoidProfile.State(drive.getRange(), state.velocity),
                        goal);

        profiledPIDController.calculate(drive.getRange());

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            System.err.println("NO ALLIANCE, CANT AUTO AMP");
            return initialSpeeds;
        }

        return new ChassisSpeeds(
                initialSpeeds.vxMetersPerSecond,
                (alliance.get() == DriverStation.Alliance.Red ? 1 : -1)
                        * (profiledPIDController.calculate(drive.getRange())
                                + profiledPIDController.getSetpoint().velocity),
                initialSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void reset() {
        state =
                new TrapezoidProfile.State(
                        drive.getRange(), drive.getFieldRelativeSpeeds().vyMetersPerSecond);
        profiledPIDController.reset(state);
    }
}
