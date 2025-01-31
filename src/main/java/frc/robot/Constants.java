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
                        CENTER_TO_MODULE.get(FL),
                        CENTER_TO_MODULE.get(FR),
                        CENTER_TO_MODULE.get(BL),
                        CENTER_TO_MODULE.get(BR));
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

    public static final class Elevator {
        public static final double L2_EXTENSION = 5; // TODO get exact value
        public static final double L3_EXTENSION = 6; // TODO get exact value
        public static final double L4_EXTENSION = 7; // TODO get exact value

        public static final double CURRENT_LIMIT = 1; // TODO config

        public static final InvertedValue LEFT_INVERT =
                InvertedValue.Clockwise_Positive; // TODO ensure that this is correct
        public static final InvertedValue RIGHT_INVERT =
                InvertedValue.CounterClockwise_Positive; // TODO ensure that this is correct

        public static final double P = 180; // TODO config
    }

    public static final class Wrist {
        public static final int CAN_ID = -1; // TODO CONFIG
        public static double OPEN_LOOP_RAMP;
        public static double GEAR_RATIO;
        public static int CURRENT_LIMIT;



    public static final class EndEffector {
        public static final int CAN_ID = 9; // TODO CONFIG
        public static final double INTAKE_SPEED = .5; // TODO CONFIG
    }
}
}
