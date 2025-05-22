package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class OIConstants {

    public static final double XBOX_STICK_DEADBAND = 0.06;

    //    public static final CommandJoystick DRIVE_CONTROLLER = new CommandJoystick(0);
    public static final CommandXboxController DRIVE_CONTROLLER = new CommandXboxController(0);
    public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(1);

    public static final Trigger TELEOP = new Trigger(DriverStation::isTeleop);
    public static final Trigger UNBOUND = new Trigger(() -> false);

    public static final Function<Double, Double> INPUT_CURVE = (x) -> .8 * x + .2 * (x * x * x);
    public static final Function<Double, Double> SPIN_CURVE = (x) -> (x * x * x);

    public static final class Drive {
        public enum RotationMode {
            HeadingControl, // The rotation stick controls the heading of the robot
            OmegaControl // The rotation stick controls the angular velocity of the robot
        }

        public static double TRANSLATION_DEADBAND = 0.05;
        public static double ROTATION_DEADBAND = 0.05;

        public static final Function<Double, Double> TRANSLATION_INPUT_CURVE =
                (x) -> .8 * x + .2 * (x * x * x);
        public static final Function<Double, Double> SPIN_CURVE = (x) -> (x * x * x);

        public static double STICK_DEADBAND = 0.08;

        // Values are in percents, we have full power
        private static final double SPIN_SPEED = Constants.compBot() ? .32 : .4;
        private static final double SUPER_SPIN = 1.0;
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

        public static final DoubleSupplier X_TRANSLATION = () -> -DRIVE_CONTROLLER.getLeftY();
        public static final DoubleSupplier Y_TRANSLATION = () -> -DRIVE_CONTROLLER.getLeftX();

        public static final DoubleSupplier ROTATION_SPEED = () -> -DRIVE_CONTROLLER.getRightX();

        public static final DoubleSupplier HEADING_CONTROL = () -> Double.NaN;
        //                        0 * Math.hypot(DRIVE_CONTROLLER.getLeftY(),
        // DRIVE_CONTROLLER.getLeftX()) > .5
        //                                ? Math.toDegrees(Math.atan2(
        //                                                -DRIVE_CONTROLLER.getLeftY(),
        //                                                -DRIVE_CONTROLLER.getLeftX()))
        //                                        - 90
        //                                : Double.NaN;

        // Drive on the fly modes
        public static final Trigger RESET_GYRO = DRIVE_CONTROLLER.back();
        public static final Trigger X_BREAK = UNBOUND;
        public static final Trigger COAST = UNBOUND;
        public static final Trigger BRAKE = UNBOUND;

        public static final Trigger ALIGN_LEFT = DRIVE_CONTROLLER.leftBumper();
//                new Trigger(() -> DRIVE_CONTROLLER.getHID().getPOV() > 180);

        public static final Trigger ALIGN_RIGHT = DRIVE_CONTROLLER.rightBumper();
//                new Trigger(() -> DRIVE_CONTROLLER.getHID().getPOV() < 180
//                        && DRIVE_CONTROLLER.getHID().getPOV() > 0);
    }

    public static final class Overrides {
        //        private static final ShuffleboardTab TAB = Shuffleboard.getTab("Overrides");

        //        @SuppressWarnings("resource")
        //        public static final BooleanSupplier INTAKE_OVERRIDE =
        //                DashboardHelpers.genericEntrySupplier(
        //                                TAB.add("intake", false)
        //                                        .withWidget(BuiltInWidgets.kBooleanBox)
        //                                        .getEntry(),
        //                                false,
        //                                NetworkTableType.kBoolean)
        //                        ::get;
        //
        //        public static final BooleanSupplier ARM_OVERRIDE =
        //                DashboardHelpers.genericEntrySupplier(
        //                                TAB.add("Arm", false)
        //                                        .withWidget(BuiltInWidgets.kBooleanBox)
        //                                        .getEntry(),
        //                                false,
        //                                NetworkTableType.kBoolean)
        //                        ::get;
    }

    public static final class Intake {
        public static final Trigger HANDOFF = UNBOUND;
        public static final Trigger ALGAE_REMOVAL = UNBOUND;
        public static final Trigger REVERSE = OPERATOR_CONTROLLER.leftBumper();
        public static final Trigger SHOOT_CORAL = OPERATOR_CONTROLLER.rightBumper();
        public static final Trigger INTAKE_ALGAE = OPERATOR_CONTROLLER.leftTrigger();
        public static final Trigger EJECT_ALGAE = OPERATOR_CONTROLLER.rightTrigger();
    }

    public static final class Wrist {
        public static final DoubleSupplier MANUAL =
                () -> MathUtil.applyDeadband(-OPERATOR_CONTROLLER.getRightY(), .1);
    }

    public static final class Elevator {
        public static final DoubleSupplier MANUAL =
                () -> .3 * MathUtil.applyDeadband(-OPERATOR_CONTROLLER.getLeftY(), .1);
    }

    public static final class SuperStructure {
        public static final Trigger SCORE = DRIVE_CONTROLLER.rightTrigger();

        public static final Trigger L1 = OPERATOR_CONTROLLER.povDown();
        public static final Trigger L2 = OPERATOR_CONTROLLER.povLeft();
        public static final Trigger L3 = OPERATOR_CONTROLLER.povUp();
        public static final Trigger L4 = OPERATOR_CONTROLLER.povRight();

        public static final Trigger HANDOFF = OPERATOR_CONTROLLER.y();
        public static final Trigger TEMP_L4_DUNK = OPERATOR_CONTROLLER.back();

        public static final Trigger KICK_BOTTOM_ALGAE = OPERATOR_CONTROLLER.a();
        public static final Trigger KICK_TOP_ALGAE = OPERATOR_CONTROLLER.b();

        public static final Trigger MANUAL_MODE = new Trigger(
                () -> Elevator.MANUAL.getAsDouble() != 0 || Wrist.MANUAL.getAsDouble() != 0);
    }

    public static final class Winch {
        public static final Trigger WINCH_MAN_UP = UNBOUND;//.button(7);
        public static final Trigger WINCH_MAN_DOWN = UNBOUND;// DRIVE_CONTROLLER.button(8);

        public static final Trigger FUNNEL_UP = UNBOUND;
        public static final Trigger FUNNEL_DOWN = UNBOUND;
    }

    public static final class Climber {
        public static final Trigger DEPLOY_CLIMBER = UNBOUND;//DRIVE_CONTROLLER.button(4); // TODO BIND
        public static final Trigger RESTOW_CLIMBER = UNBOUND;//DRIVE_CONTROLLER.button(6);

        public static final Trigger CLIMBER_UP_MAN = UNBOUND;//DRIVE_CONTROLLER.button(9);
        public static final Trigger CLIMBER_DOWN_MAN = UNBOUND;//DRIVE_CONTROLLER.button(10);
    }

    //    public static final class TestMode {
    //        public static final Trigger ZERO_ABS_ENCODERS = UNBOUND;
    //
    //        // TODO, Move these to shuffleboard buttons in their respective dashboards.
    //        public static final class SysId {
    //            public static final class Arm {
    //                public static final Trigger ARM_TEST =
    //                        new Trigger(DriverStation::isTest).and(TEST_CONTROLLER.povLeft());
    //                public static final Trigger QUASISTATIC_FWD =
    // ARM_TEST.and(TEST_CONTROLLER.y());
    //                public static final Trigger QUASISTATIC_REV =
    // ARM_TEST.and(TEST_CONTROLLER.x());
    //                public static final Trigger DYNAMIC_FWD = UNBOUND;
    // ARM_TEST.and(TEST_CONTROLLER.b());
    //               public static final Trigger DYNAMIC_FWD =
    // ARM_TEST.and(TEST_CONTROLLER.b());
    //                public static final Trigger DYNAMIC_REV = ARM_TEST.and(TEST_CONTROLLER.a());
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
