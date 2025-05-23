/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleUnaryOperator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Units: Unless defined otherwise, or wrapped with the wpilib units library, all values should
 * be in <a href="https://en.wikipedia.org/wiki/International_System_of_Units">SI Units</a>
 */
public final class Constants {
    public static final Mode SIM_MODE = Mode.SIM;

    public static final boolean TUNING_MODE = true;
    public static boolean DISABLE_HAL = false; // IDK What this does

    public enum Mode {
        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum Robot {
        CompBot,
        DevBot,
        SimBot
    }

    public static final Robot ROBOT = Robot.SimBot;

    public static boolean compBot() {
        return ROBOT == Robot.CompBot;
    }

    public static boolean devBot() {
        return ROBOT == Robot.DevBot;
    }

    //    public static H compDev<H>(H comp, H dev) {
    //        return ROBOT == Robot.CompBot ? comp : dev;
    //    }

    public static final double LOOP_PERIOD_SEC = frc.robot.Robot.defaultPeriodSecs;

    public static final class Drive {
        public static final int PIGEON_ID = 14;
        public static final int FUSION_TIME_OF_FLIGHT_ID = 0;

        public static final COTSSwerveConstants CHOSEN_MODULE =
                COTSSwerveConstants.SDSMK4i(
                        compBot()
                                ? COTSSwerveConstants.driveGearRatios.SDSMK4i_L3
                                : COTSSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.25);
        public static final double WHEEL_BASE = Units.inchesToMeters(24.25);
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        public static final int FL = 0; // Front Left Module Index
        public static final int FR = 1; // Front Right Module Index
        public static final int BL = 2; // Back Left Module Index
        public static final int BR = 3; // Back Right Module Index

        public static final List<Translation2d> CENTER_TO_MODULE =
                Arrays.asList(
                        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        CENTER_TO_MODULE.get(FL),
                        CENTER_TO_MODULE.get(FR),
                        CENTER_TO_MODULE.get(BL),
                        CENTER_TO_MODULE.get(BR));

        /* Current Limits
         *
         * Current Limits attempt to prevent the motor from burning out under a stall condition, and prevent the breaker from being tripped.
         *
         * The current limits are from the controller to the motor, and aren't necessarily the same as the current coming from the battery.
         *
         * The smart current limit scales the current limit based on the speed of the motor (to make it better for closed loop control iirc),
         * Secondary limit turns off the motor until the current falls below the limit
         *
         * Because of how fuses work, they can sustain current draw above their rated value for short periods of time before tripping, meaning these values do have meaning above 40
         */
        public static final class CurrentLimits {
            public static class Spark {

                public static final int DRIVE_SMART_CURRENT_LIMIT = 65;
                public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 80;
            }

            public static class Kraken {
                public static final int DRIVE_STATOR = 120;
            }
        }

        public static final int ANGLE_SMART_CURRENT_LIMIT = 35;
        public static final int ANGLE_SECONDARY_CURRENT_LIMIT = 50;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = compBot() ? .05 : 0.0035;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;

        public static final double DRIVE_KP = compBot() ? 0.73983 : 0.028215;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;

        /* Drive Motor Characterization Values in volts*/
        public static final double DRIVE_KS = compBot() ? 0.179124 : (0.19714);
        public static final double DRIVE_KV = compBot() ? 0.7389575 : (2.6198);
        public static final double DRIVE_KA = compBot() ? 0.14679 : (0.59488);

        /* Drive Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.6; // TODO: This must be tuned to specific robot

        /**
         * Theoretical value can be calculated by dividing MAX_LINEAR_SPEED by the distance between the center of
         * rotation and the wheel.
         */
        public static final double MAX_ANGULAR_VELOCITY =
                10.0; // TODO: This must be tuned to specific robot

        /* Brake Modes */
        public static final boolean ANGLE_BRAKE_MODE = false;
        public static final boolean DRIVE_BRAKE_MODE = true;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 6;
            public static final int ANGLE_MOTOR_ID = 7;
            public static final int CAN_CODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET =
                    Rotation2d.fromDegrees(ROBOT == Robot.CompBot ? 148.89 : 0);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET =
                    Rotation2d.fromDegrees(ROBOT == Robot.CompBot ? 113.56 : 0);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CAN_CODER_ID = 4;
            public static final Rotation2d ANGLE_OFFSET =
                    Rotation2d.fromDegrees(ROBOT == Robot.CompBot ? 177.19 : 0);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 12;
            public static final int ANGLE_MOTOR_ID = 13;
            public static final int CAN_CODER_ID = 5;
            public static final Rotation2d ANGLE_OFFSET =
                    Rotation2d.fromDegrees(ROBOT == Robot.CompBot ? 129.73 : 0);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        public static final double MASS = Units.lbsToKilograms(115 + 20);
        public static final double MOI =
                (1 / 12.0)
                        * MASS
                        * (WHEEL_BASE * WHEEL_BASE
                                + TRACK_WIDTH * TRACK_WIDTH); // TODO: EMPIRICALLY MEASURE MOI
        public static final double MAX_MODULE_ANGULAR_VELOCITY =
                Units.rotationsToRadians(10.0); // CONFIG

        public static final RobotConfig PHYSICAL_CONSTANTS =
                new RobotConfig(
                        MASS,
                        MOI,
                        new ModuleConfig(
                                WHEEL_CIRCUMFERENCE / (2 * Math.PI),
                                MAX_SPEED,
                                1.2, // TODO, MEASURE WHEEL COEFFICIENT OF FRICTION,
                                DCMotor.getKrakenX60Foc(1).withReduction(DRIVE_GEAR_RATIO),
                                60,
                                1),
                        CENTER_TO_MODULE.get(FL),
                        CENTER_TO_MODULE.get(FR),
                        CENTER_TO_MODULE.get(BL),
                        CENTER_TO_MODULE.get(BR));
    }

    public static final class Elevator {

        public static final int RIGHT_ID = 20;
        public static final int LEFT_ID = 21;

        public static final double MIN_POS = compBot() ? 0 : .001;
        public static final double MAX_POS = compBot() ? 1.489501953125 : 1.48;

        public static final double TOLERANCE = compBot() ? 0.005 : 0.01;

        public static final double OPEN_LOOP_RAMP = .25;
        public static final int CURRENT_LIMIT = 60;
        public static final double ELEVATOR_GEAR_RATIO = compBot() ? 5 : 12;
        public static final double SPROCKET_CIRCUMFERENCE =
                Units.inchesToMeters(.25) * (compBot() ? 22 : 24);

        public static final InvertedValue LEFT_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue RIGHT_INVERT = InvertedValue.Clockwise_Positive;

        public static final double MAX_VEL = compBot() ? 6 : 2;
        public static final double MAX_ACCEL = compBot() ? 12 : 3;

        public static final class RightGains {
            public static final double KS = 0.26266;
            public static final double KV = 4.4755;
            public static final double KA = 0.69186;
            public static final double KG = 0.38353;
            public static final double KP = 5; // 4.3951;
            public static final double KD = 2; // 952.83;
            public static final double KI = 0;
        }

        public static final class LeftGains {
            public static final double KS = 0.23462;
            public static final double KV = 4.5305;
            public static final double KA = 0.84246;
            public static final double KG = 0.24188;
            public static final double KP = 5; // 4.2941;
            public static final double KD = 2; // 905.76;
            public static final double KI = 0;
        }

        public static final class KrakenGains {
            public static final double KS = 0.11757;
            public static final double KV = 0.12729;
            public static final double KA = 0.0055511;
            public static final double KG = 0.26856;
            public static final double KP = 4 * 12; // 75.313;
            public static final double KD = 0; // 0.84699;
        }
    }

    public static final class Wrist {
        public static final int CAN_ID = 30;
        public static final int ABS_ENCODER_ID = 31;
        public static final double WRIST_GEAR_RATIO = (9.0) * (5.0) * (54.0 / 16.0);

        public static final double OPEN_LOOP_RAMP = .25;
        public static final int CURRENT_LIMIT = 60;
        public static final boolean INVERTED = false;

        public static final double MAX_POS = Units.degreesToRadians(compBot() ? 94 : 90);
        public static final double MIN_POS = Units.degreesToRadians(-90);

        public static final double ABS_ENCODER_ZERO = Math.toRadians(306.71 + 90);
        public static final double TOLERANCE = Units.degreesToRadians(2.5);

        public static final double MAX_VELOCITY =
                Constants.compBot() ? Units.rotationsToRadians(4) : Units.degreesToRadians(180);
        public static final double MAX_ACCEL =
                Constants.compBot() ? Units.rotationsToRadians(8) : Units.degreesToRadians(360);

        public static final class PidGains {
            public static final double KP = 2;
            public static final double KI = 0;
            public static final double KD = 0; // 524.32
        }

        // bad cuz backlash
        public static final class WristGains {
            public static final double KS = 0.0080265;
            public static final double KV = 2.8461;
            public static final double KA = 0.66332;
            public static final double KG = 0.2288;
        }

        public static final class KrakenGains {
            public static final double KS = 0.3576;
            public static final double KV = 2.5726 * (2 * Math.PI);
            public static final double KA = 0.059493 * (2 * Math.PI);
            public static final double KG = 0.16157;
            public static final double KP = 6 * 12 * (2 * Math.PI);
            public static final double KD = 0; // 0.70647
        }
    }

    // Reset Elevator To Bottom before reset code
    public static final class SuperstructureSetpoints {
        public static final SuperstructureState STOW =
                new SuperstructureState(.001, Math.toRadians(75));

        public static final SuperstructureState WRIST_TRANSIT =
                new SuperstructureState(0, Math.toRadians(70));

        public static final SuperstructureState L1 =
                new SuperstructureState(.2, Math.toRadians(60));
        public static final SuperstructureState L2 =
                new SuperstructureState(.48, Math.toRadians(60));
        public static final SuperstructureState L3 =
                new SuperstructureState(.85, Math.toRadians(60));
        public static final SuperstructureState L4 =
                new SuperstructureState(1.48, Math.toRadians(57.5)); // Value is not with Drop

        public static final SuperstructureState KICK_LOW_ALGAE =
                new SuperstructureState(.48, Math.toRadians(42));
        public static final SuperstructureState KICK_HIGH_ALGAE =
                new SuperstructureState(.88, Math.toRadians(40));

        public static final SuperstructureState L4_PLOP =
                new SuperstructureState(L4.elevatorPosition, Math.toRadians(30));

        public static final SuperstructureState PICKUP_LOW_ALGAE =
                new SuperstructureState(.48 + .2, Math.toRadians(60));
        public static final SuperstructureState PICKUP_HIGH_ALGAE =
                new SuperstructureState(.88 + .2, Math.toRadians(60));

        public static final List<StateWithMode> L4_DUNK =
                List.of(
                        SetpointMode.WRIST_SYNC.withState(
                                new SuperstructureState(L4.elevatorPosition, Math.toRadians(0))),
                        SetpointMode.WRIST_SYNC.withState(
                                new SuperstructureState(1.31, Math.toRadians(0))),
                        SetpointMode.WRIST_SYNC.withState(
                                new SuperstructureState(1.0, Math.toRadians(70))));

        public static final SuperstructureState HANDOFF =
                new SuperstructureState(.001, Math.toRadians(94));

        public static final double ELEVATOR_MAX_TO_SKIP_TRANSIT = .15;

        public enum SetpointMode {
            WRIST_FIRST,
            WRIST_LAST,
            WRIST_SYNC;

            public StateWithMode withState(SuperstructureState state) {
                return new StateWithMode(state, this);
            }
        }

        public record StateWithMode(SuperstructureState state, SetpointMode mode) {}

        public record SuperstructureState(double elevatorPosition, double wristRotation) {
            public TrapezoidProfile.State getElevatorState(double velocity) {
                return new TrapezoidProfile.State(elevatorPosition, velocity);
            }

            public TrapezoidProfile.State getElevatorState() {
                return getElevatorState(0);
            }

            public TrapezoidProfile.State getWristState(double velocity) {
                return new TrapezoidProfile.State(wristRotation, velocity);
            }

            public TrapezoidProfile.State getWristState() {
                return getWristState(0);
            }
        }
    }

    public static final class Intake {
        public static final int CAN_ID = 40;
        public static final boolean INVERTED = compBot() ? true : false;
        public static final int CURRENT_LIMIT = compBot() ? 80 : 25;
        public static final double HANDOFF_SPEED = compBot() ? .8 : .5;
        public static final double REVERSE_SPEED = -.15;

        public static final double ALGAE_HOLD = -.4; // TODO CONFIG
        public static final double ALGAE_EJECT = .4; // TODO CONFIG

        public static final double ALGAE_REMOVAL = .5; // TODO CONFIG
        public static final double SHOOT_CORAL = .5;
        public static final double L1 = .3;

        public static final double L4_PLOP = .4;
    }

    public static final class Winch {
        public static final int ID = 50;

        public static final int CURRENT_LIMIT = 25;

        public static final double MATCH_FUNNEL_UP = 1.35;
        public static final double MATCH_FUNNEL_DOWN = 0;
        public static final double PIT_FUNNEL_STOW = 0;
        public static final double EPSILON = 0.05;

        public static final double WINCH_GEAR_RATIO = 25;

        public static final double KP = 2;
        public static final double MAX_OUT = .6;
    }

    public static final class Climber {
        public static final int RIGHT_ID = 60;
        public static final int LEFT_ID = 61;

        public static final double MAX_POS = Math.toRadians(270);
        public static final double MIN_POS = Math.toRadians(0);

        public static final double ABS_ENCODER_ZERO =
                Units.rotationsToRadians(0.9178690729467268 - .75);
        public static final int ABS_ENCODER_DIO_PORT = 1;

        public static final double DEPLOY_POSITION = Math.toRadians(105); // TODO CONFIG
        public static final double CLIMB_POSITION = Math.toRadians(230);
        public static final double RESTOW_POSITION = Math.toRadians(270);

        public static final double STARTING_POSITION = Math.toRadians(270);

        public static final double EPSILON = Math.toRadians(1);

        public static final double CLIMBER_GEAR_RATIO = (9 * 5 * 3) * (24.0 / 12.0);

        public static final class UnloadedGains {
            public static final double KS = 0.17029;
            public static final double KV = 5.1394;
            public static final double KA = 0.064502;

            public static final double KP = 24; // 65.595;
        }

        public static final class LoadedGains {
            public static final double KS = 0.17029;
            public static final double KV = 5.1394;
            public static final double KA = 0.064502;

            public static final double KP = 100; // 65.595;
        }

        public static final double MAX_VEL_UNLOADED = Units.degreesToRadians(180 * .6);
        public static final double MAX_ACCEL_UNLOADED = Units.degreesToRadians(360 * .6);

        public static final double MAX_VEL_LOADED = .5;
        public static final double MAX_ACCEL_LOADED = 1;
    }

    public static final class Vision {

        public static final List<RobotCamera> CAMERAS =
                List.of(
                        new RobotCamera( // FRONT LEFT
                                "Blitz_2_OV2311",
                                new Transform3d(
                                        new Translation3d(
                                                // TODO, VERIFY
                                                Inches.of(11.104033),
                                                Inches.of(10.101652),
                                                Inches.of(9.504322)),
                                        new Rotation3d(
                                                Degrees.of(0), Degrees.of(-20), Degrees.of(-20)))),
                        new RobotCamera( // FRONT RIGHT
                                "Blitz_1_OV2311",
                                new Transform3d(
                                        new Translation3d(
                                                // TODO, VERIFY
                                                Inches.of(11.104033),
                                                Inches.of(-10.101652),
                                                Inches.of(9.504322)),
                                        new Rotation3d(
                                                Degrees.of(0), Degrees.of(-20), Degrees.of(20)))));

        public record RobotCamera(String name, Transform3d pose) {}
    }

    public static final class Auto {
        public static final class Timings {
            public static final double STOW_TO_L4_READY = 1.25; // IDK ACTUAL TIMINGS
        }
    }
}
