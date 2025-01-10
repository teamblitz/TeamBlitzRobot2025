/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
 * <p>Units: Distance in meters Rotation in radians Time in seconds
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

    public static final double LOOP_PERIOD_SEC = frc.robot.Robot.defaultPeriodSecs;

    public static final class Drive {
        public static final class NoteAssist {
            public static double ACTIVATION_RANGE = Units.degreesToRadians(45);
            public static DoubleUnaryOperator ACTIVATION_FUNCTION =
                    (x) ->
                            Units.degreesToRadians(
                                    Math.exp(
                                            -Math.pow(
                                                    Units.radiansToDegrees(x) / 40,
                                                    4))); // Output a value between 0 and 1, 0 means
            // no assist, 1 means full assist
        }

        public static final int PIGEON_ID = 14;
        public static final int FUSION_TIME_OF_FLIGHT_ID = 0;
        public static final boolean USE_PIGEON = true;

        public static final COTSSwerveConstants CHOSEN_MODULE =
                COTSSwerveConstants.SDSMK4i(
                        compBot()
                                ? COTSSwerveConstants.driveGearRatios.SDSMK4i_L3
                                : COTSSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH =
                Units.inchesToMeters(24.75 + (compBot() ? -.25 : 0));
        public static final double WHEEL_BASE =
                Units.inchesToMeters(24.75 + (compBot() ? -.25 : 0));
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
        public static final double ANGLE_KP = compBot() ? .004 : 0.0035;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_KF = 0.0; // For now, should remain zero

        /* Drive Motor PID Values */
        /*
         * Some possible gains for kp
         * kp : 0.0016 or more likely 0.028215
         * try both or else just guess and check ig
         * .06 something might, but that is quite high
         */
        public static final double DRIVE_KP = compBot() ? 3.8342 : 0.028215;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;

        /* Drive Motor Characterization Values in volts*/
        public static final double DRIVE_KS = compBot() ? 0.11193 : (0.19714);
        public static final double DRIVE_KV = compBot() ? 2.5025 : (2.6198);
        public static final double DRIVE_KA = compBot() ? 0.55717 : (0.59488);

        /* Drive Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.6; // TODO: This must be tuned to specific robot

        /**
         * Radians per Second
         *
         * <p>Can likely be figured out using an equation. Or we can just tornado spin it and see
         * what happens.
         *
         * <p>public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
         * MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
         * DRIVETRAIN_WHEELBASE_METERS / 2.0);
         *
         * <p>Assuming our robot can still go at 4.6 meters per second (which it can't, this value
         * was taken when we had like nothing on our robot, we can go 10.35 radians per second while
         * spinning
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
                    Rotation2d.fromDegrees(ROBOT == Robot.CompBot ? 103.711 : 359.077);
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
                    Rotation2d.fromDegrees(ROBOT == Robot.CompBot ? -37.617 : 269.736);
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
                    Rotation2d.fromDegrees(ROBOT == Robot.CompBot ? -71.895 : 1.582);
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
                    Rotation2d.fromDegrees(ROBOT == Robot.CompBot ? -105.117 : 89.253);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        public static final double MASS = Units.lbsToKilograms(150);
        public static final double MOI =
                1
                        / 12.
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
                                1.0, // TODO, MEASURE WHEEL COEFICENT OF FRICTION,
                                compBot() ? DCMotor.getKrakenX60Foc(1) : DCMotor.getNEO(1),
                                CurrentLimits.Kraken
                                        .DRIVE_STATOR, // TODO, WRONG PROBABLY, might be SUPPLY
                                // limit, which we don't actualy set.,
                                1),
                        TRACK_WIDTH,
                        WHEEL_BASE);
    }

    public static final class Arm {
        public static final double MIN_ROT = Units.degreesToRadians(-10); // TODO, TUNE THIS
        public static final double MAX_ROT = Units.degreesToRadians(103);
        public static final double MAX_STAGE = Units.degreesToRadians(10); // TODO, tune this

        public static final double STARTING_POS =
                Units.degreesToRadians(5.63); // 3.349 degrees, alternativly 5.63
        public static final double ABS_ENCODER_OFFSET = Units.degreesToRadians(24.91 - 90);

        public static final double MAX_VELOCITY = Units.degreesToRadians(compBot() ? 120 : 150);
        public static final double MAX_ACCELERATION =
                Units.degreesToRadians(compBot() ? 240 : 180); // prev 240

        public static final int ARM_ROT_LEADER = 16;
        public static final int ARM_ROT_FOLLOWER = 15;

        public static final int ABS_ENCODER = 0;
        public static final int QUAD_A = 1;
        public static final int QUAD_B = 2;

        public static final double OPEN_LOOP_RAMP = 1;

        public static final int CURRENT_LIMIT = 60;

        public static final class FeedForwardConstants {
            public static final double KS = 0.41881;
            public static final double KV = 2.4974;
            public static final double KA = 1.4009;
            public static final double KG = 0.92004;
        }

        public static final class PidConstants {
            public static final double P = 1.1; // 0.19905 from sysid
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final double GEAR_RATIO =
                ROBOT == Robot.CompBot ? (3 * 3 * 4) * (58.0 / 12.0) : (3 * 3 * 4) * (64.0 / 12.0);

        public static final class Positions {
            public static final double INTAKE =
                    compBot()
                            ? Units.degreesToRadians(-2)
                            : STARTING_POS + Units.degreesToRadians(-5);
            public static final double CLIMB = MAX_STAGE;
            public static final double TRANSIT_NORMAL = Units.degreesToRadians(60);
            public static final double AMP_FRONT = Units.degreesToRadians(103);
            public static final double AMP_BACK = Units.degreesToRadians(60);

            public static final double SPEAKER_SUB_FRONT =
                    Units.degreesToRadians(30 + (compBot() ? 2 : 20));
            public static final double SPEAKER_SUB_SIDE =
                    Units.degreesToRadians(46 + 2); // actually bot distance
            public static final double SPEAKER_PODIUM =
                    Units.degreesToRadians(47 + (compBot() ? 2 : 20));
        }
    }

    public static class Intake {
        public static final int CURRENT_LIMIT = 50;

        public static final class Spark {
            public static final int MOTOR_ID = 19;
        }
    }

    public static class Shooter {

        public static final double MAX_VELOCITY =
                27.330532277373607; // the theoretical top max velocity

        public static class Spark {

            public static final int SPARK_TOP = 22;
            public static final int SPARK_BOTTOM = 23;

            public static final double PID_TOP_P =
                    compBot() ? 0.01182 : 0.013715; // TODO SET Was 0.03
            public static final int PID_TOP_I = 0; // TODO SET
            public static final int PID_TOP_D = 0; // TODO SET

            public static final double PID_BOTTOM_P =
                    compBot() ? 0.0074161 : 0.013715; // TODO SET was 0.03
            public static final int PID_BOTTOM_I = 0; // TODO SET
            public static final int PID_BOTTOM_D = 0; // TODO SET

            public static final double FF_TOP_KS = compBot() ? 0.12871 : 0.043031; // TODO SET
            public static final double FF_TOP_KV = compBot() ? 0.43436 : 0.39694; // TODO SET
            public static final double FF_TOP_KA = compBot() ? 0.079988 : 0.086385;

            public static final double FF_BOTTOM_KS = compBot() ? 0.15595 : 0.043031; // TODO SET
            public static final double FF_BOTTOM_KV = compBot() ? 0.42368 : 0.39694; // TODO SET
            public static final double FF_BOTTOM_KA = compBot() ? 0.060884 : 0.086385;

            public static final double GEAR_RATIO = (22.0 / 24.0);

            public static final double VELOCITY_FACTOR_RPM_TO_MPS =
                    GEAR_RATIO * (1.0 / 60.0) * (Math.PI * 2 * Units.inchesToMeters(2));

            public static final double POSITION_FACTOR_ROT_TO_M =
                    GEAR_RATIO * (Math.PI * 2 * Units.inchesToMeters(2));
        }

        public static class Talon {

            public static final int TALON_TOP = 22;
            public static final int TALON_BOTTOM = 23;
        }

        public static final int CURRENT_LIMIT = 60;

        public static class AutoShootConstants {
            public static final Transform2d BOT_TO_CENTER_OF_ROTATION =
                    new Transform2d(
                            Units.inchesToMeters(-(15 - 4)),
                            Units.inchesToMeters(4),
                            Rotation2d.fromRadians(0));
            public static final Transform2d CENTER_OF_ROTATION_TO_SHOOTER =
                    new Transform2d(
                            Units.inchesToMeters(-27),
                            Units.inchesToMeters(4),
                            Rotation2d.fromRadians(0));

            public static final double SHOOT_ANGLE_OFFSET = Units.degreesToRadians(20);

            public static final InterpolatingDoubleTreeMap ANGLE_TREE_MAP =
                    new InterpolatingDoubleTreeMap();

            static {
                ANGLE_TREE_MAP.put(1.19, Units.degreesToRadians(29));
                ANGLE_TREE_MAP.put(1.365, Units.degreesToRadians(35));
                ANGLE_TREE_MAP.put(1.7, Units.degreesToRadians(39));
                ANGLE_TREE_MAP.put(2.03, Units.degreesToRadians(43));
                ANGLE_TREE_MAP.put(2.225, Units.degreesToRadians(46));
                ANGLE_TREE_MAP.put(2.48, Units.degreesToRadians(47));
                ANGLE_TREE_MAP.put(2.66, Units.degreesToRadians(48.5));
                ANGLE_TREE_MAP.put(2.90, Units.degreesToRadians(50.5));
                ANGLE_TREE_MAP.put(3.0, Units.degreesToRadians(51));
                ANGLE_TREE_MAP.put(2.314, Units.degreesToRadians(52.25));

                //                angleTreeMap.put(1.45, Units.degreesToRadians(40));
                //                angleTreeMap.put(1.77, Units.degreesToRadians(43));
                //                angleTreeMap.put(2.02, Units.degreesToRadians(47));
                //                angleTreeMap.put(2.08, Units.degreesToRadians(48));
                //                angleTreeMap.put(2.4, Units.degreesToRadians(49));
                //                angleTreeMap.put(2.67, Units.degreesToRadians(51.8));
            }

            public static final InterpolatingDoubleTreeMap SHOOT_VELOCITY_TREE_MAP =
                    new InterpolatingDoubleTreeMap();

            static {
                SHOOT_VELOCITY_TREE_MAP.put(10.0, MAX_VELOCITY * .8);
                SHOOT_VELOCITY_TREE_MAP.put(2.0, MAX_VELOCITY * .8);
                SHOOT_VELOCITY_TREE_MAP.put(0.0, MAX_VELOCITY * .6);
            }

            public static final Pose3d GOAL_POSE_BLUE =
                    new Pose3d(0.2269, 5.5526, 2.0451, new Rotation3d());
            public static final Pose3d GOAL_POSE_RED =
                    new Pose3d(16.3062, 5.5556, 2.0446, new Rotation3d());

            public static final double SHOOT_VELOCITY = 23.6 * .8;
        }
    }

    public static class Climber {
        public static final int LEFT_MOTOR_ID = 17;
        public static final int RIGHT_MOTOR_ID = 18;

        public static final double CURRENT_LIMIT = 80;

        public static final double MAX_EXTENSION = .46; // TODO set

        public static final double GEAR_RATIO = (15.0 / 1.0);
        public static final double SPOOL_DIAMETER = Units.inchesToMeters(.75);
        public static final InvertedValue LEFT_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RIGHT_INVERT = InvertedValue.CounterClockwise_Positive;

        public static final double P = 180;
    }

    public static final class AutoConstants {

        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 5;
        public static final double PY_CONTROLLER = 5;
        public static final double P_THETA_CONTROLLER = 5;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double MAX_MODULE_SPEED = 3; // M/S

        public static final com.pathplanner.lib.config.PIDConstants TRANSLATION_PID =
                new com.pathplanner.lib.config.PIDConstants(5, 0, 0);
        public static final com.pathplanner.lib.config.PIDConstants ROTATION_PID =
                new com.pathplanner.lib.config.PIDConstants(5, 0, 0);

        public enum StartingPosition {
            LEFT(60),
            RIGHT(-60),
            CENTER(0);

            public final double angle;

            StartingPosition(double angle) {
                this.angle = angle;
            }
        }
    }

    public static final class Networking {
        public static final String JETSON_IP_ADDRESS = "10.20.83.130";
        public static final int PORT = 5810;
        public static final int INTERVAL = 5;
    }
}
