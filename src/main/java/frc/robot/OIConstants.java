package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.DashboardHelpers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class OIConstants {

    public static final CommandJoystick DRIVE_CONTROLLER = new CommandJoystick(0);
    public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(1);
    public static final boolean TEST_CONTROLS = false;
    public static final CommandXboxController TEST_CONTROLLER =
            TEST_CONTROLS ? new CommandXboxController(2) : null;

    public static final Trigger TELEOP = new Trigger(DriverStation::isTeleop);
    public static final Trigger UNBOUND = new Trigger(() -> false);

    public static final Function<Double, Double> INPUT_CURVE = (x) -> .8 * x + .2 * (x * x * x);
    public static final Function<Double, Double> SPIN_CURVE = (x) -> (x * x * x);

    public static final class Drive {
        public static double STICK_DEADBAND = 0.08;

        // Values are in percents, we have full power
        private static final double SPIN_SPEED = Constants.compBot() ? .32 : .4;
        private static final double SLOW_SPEED = .3;
        public static final double NORMAL_SPEED = .6;
        public static final double FAST_SPEED = 1;

        private static final SlewRateLimiter DRIVE_MULTIPLIER_LIMITER =
                new SlewRateLimiter(.25); // Todo, try without this?

        //        private static final DoubleSupplier DRIVE_MULTIPLIER =
        //                () ->
        //                        NORMAL_SPEED
        //                                + DRIVE_CONTROLLER.getLeftTriggerAxis()
        //                                        * (SLOW_SPEED - NORMAL_SPEED)
        //                                + DRIVE_CONTROLLER.getRightTriggerAxis()
        //                                        * (FAST_SPEED - NORMAL_SPEED);

        private static final DoubleSupplier DRIVE_MULTIPLIER =
                () -> (DRIVE_CONTROLLER.getHID().getRawButton(1) ? FAST_SPEED : NORMAL_SPEED);

        public static final DoubleSupplier X_TRANSLATION =
                () -> INPUT_CURVE.apply(-DRIVE_CONTROLLER.getY()) * DRIVE_MULTIPLIER.getAsDouble();

        public static final DoubleSupplier Y_TRANSLATION =
                () -> INPUT_CURVE.apply(-DRIVE_CONTROLLER.getX()) * DRIVE_MULTIPLIER.getAsDouble();

        public static final DoubleSupplier ROTATION_SPEED =
                () -> SPIN_SPEED * SPIN_CURVE.apply(-DRIVE_CONTROLLER.getTwist());

        public static final DoubleSupplier HEADING_CONTROL = () -> Double.NaN;
        //                        0 * Math.hypot(DRIVE_CONTROLLER.getLeftY(),
        // DRIVE_CONTROLLER.getLeftX()) > .5
        //                                ? Math.toDegrees(Math.atan2(
        //                                                -DRIVE_CONTROLLER.getLeftY(),
        //                                                -DRIVE_CONTROLLER.getLeftX()))
        //                                        - 90
        //                                : Double.NaN;

        // Drive on the fly modes
        public static final Trigger RESET_GYRO = DRIVE_CONTROLLER.button(5);
        public static final Trigger X_BREAK = UNBOUND;
        public static final Trigger COAST = UNBOUND;
        public static final Trigger BRAKE = UNBOUND;
        public static final Trigger AUTO_PICKUP = DRIVE_CONTROLLER.button(2);

        public static final Trigger AIM_SPEAKER = Arm.AUTO_AIM_SPEAKER;
        public static final Trigger AMP_ASSIST = OPERATOR_CONTROLLER.b();
    }

    public static final class Intake {
        public static final Trigger FEED = OPERATOR_CONTROLLER.leftBumper();
        public static final Trigger EJECT = OPERATOR_CONTROLLER.povUp();

        public static final Trigger SCORE = DRIVE_CONTROLLER.button(3);
    }

    public static final class Shooter {
        public static final Trigger MANUAL_FEED = OPERATOR_CONTROLLER.rightBumper();
        public static final Trigger SHOOTER_AMP = UNBOUND;
        public static final Trigger EJECT = OPERATOR_CONTROLLER.povUp();
        public static final Trigger SPEED_AUTO = OPERATOR_CONTROLLER.back();
    }

    public static final class Arm {
        public static final DoubleSupplier MANUAL_ARM_SPEED =
                () -> -OPERATOR_CONTROLLER.getLeftY() * .2;

        public static final Trigger INTAKE =
                TELEOP.and(OPERATOR_CONTROLLER.a()).or(Drive.AUTO_PICKUP);

        public static final Trigger TRANSIT_STAGE =
                TELEOP.and(
                        OPERATOR_CONTROLLER
                                .povDown()
                                .or(OPERATOR_CONTROLLER.povDownLeft())
                                .or(OPERATOR_CONTROLLER.povDownRight()));

        public static final Trigger SPEAKER_SUB_FRONT =
                TELEOP.and(OPERATOR_CONTROLLER.y().and(OPERATOR_CONTROLLER.b().negate()));
        public static final Trigger SPEAKER_SUB_SIDE = UNBOUND;
        public static final Trigger SPEAKER_PODIUM = UNBOUND;

        public static final Trigger AMP_FRONT = UNBOUND; // TELEOP.and(OPERATOR_CONTROLLER.b());
        public static final Trigger AMP_BACK = TELEOP.and(OPERATOR_CONTROLLER.x());

        public static final Trigger AUTO_AIM_SPEAKER = TELEOP.and(OPERATOR_CONTROLLER.start());
    }

    public static final class Climber {
        public static final Trigger UP_BOTH = OPERATOR_CONTROLLER.leftTrigger();
        public static final Trigger DOWN_BOTH = OPERATOR_CONTROLLER.rightTrigger();
        public static final Trigger DOWN_MANUAL = OPERATOR_CONTROLLER.povRight();
    }

    public static final class Overrides {
        private static final ShuffleboardTab TAB = Shuffleboard.getTab("Overrides");

        @SuppressWarnings("resource")
        public static final BooleanSupplier INTAKE_OVERRIDE =
                DashboardHelpers.genericEntrySupplier(
                                TAB.add("Intake", false)
                                        .withWidget(BuiltInWidgets.kBooleanBox)
                                        .getEntry(),
                                false,
                                NetworkTableType.kBoolean)
                        ::get;

        public static final BooleanSupplier ARM_OVERRIDE =
                DashboardHelpers.genericEntrySupplier(
                                TAB.add("Arm", false)
                                        .withWidget(BuiltInWidgets.kBooleanBox)
                                        .getEntry(),
                                false,
                                NetworkTableType.kBoolean)
                        ::get;
    }

    //    public static final class TestMode {
    //        public static final Trigger ZERO_ABS_ENCODERS = UNBOUND;
    //
    //        // TODO, Move these to shuffleboard buttons in their respective dashboards.
    //        public static final class SysId {
    //            public static final class Arm {
    ////                public static final Trigger ARM_TEST =
    ////                        new Trigger(DriverStation::isTest).and(TEST_CONTROLLER.povLeft());
    ////                public static final Trigger QUASISTATIC_FWD =
    // ARM_TEST.and(TEST_CONTROLLER.y());
    ////                public static final Trigger QUASISTATIC_REV =
    // ARM_TEST.and(TEST_CONTROLLER.x());
    ////                public static final Trigger DYNAMIC_FWD = UNBOUND;
    // //ARM_TEST.and(TEST_CONTROLLER.b());
    //////                public static final Trigger DYNAMIC_FWD =
    // ARM_TEST.and(TEST_CONTROLLER.b());
    ////                public static final Trigger DYNAMIC_REV = ARM_TEST.and(TEST_CONTROLLER.a());
    //            }
    //
    //            public static final class Drive {
    //                public static final Trigger DRIVE_TEST =
    //                        new Trigger(DriverStation::isTest).and(TEST_CONTROLLER.povDown());
    //                public static final Trigger QUASISTATIC_FWD =
    // DRIVE_TEST.and(TEST_CONTROLLER.y());
    //                public static final Trigger QUASISTATIC_REV =
    // DRIVE_TEST.and(TEST_CONTROLLER.x());
    //                public static final Trigger DYNAMIC_FWD = DRIVE_TEST.and(TEST_CONTROLLER.b());
    //                public static final Trigger DYNAMIC_REV = DRIVE_TEST.and(TEST_CONTROLLER.a());
    //            }
    //        }
    //    }
}
