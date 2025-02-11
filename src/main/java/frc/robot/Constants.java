package frc.robot;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

// Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/* 
 * To access numbers in this file, import or statically import one of its subclasses:
 * example:
 * import static frc.robot.Constants.ControllerPorts.*;
 * import frc.robot.Constants.DriveConstants;
 */
public final class Constants {

    /* -------------- IDs -------------- */

    /** Ports used by controllers. */
    public static final class ControllerPorts {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    /* -------------- SUBSYTEM CONSTANTS -------------- */

    public static final class ClimberConstants {
        public enum ClimberPosition {
            //climber setpoints
            zero(0),
            stow(16.4),
            climb(-41.4);

            private double setpoint;
            ClimberPosition(double setpoint) {
                this.setpoint = setpoint;
            };

            //gets the angle of setpoint
            public double getAngle() {
                return setpoint;
            }
        }
    }
    
    /* -------------- DRIVETRAIN CONSTANTS -------------- */

    public static final class DriveConstants {
        /** Higher values make the robot drive more aggressively */
        public static final double TRANSLATION_SLEW = 4;
        /** Higher values make the robot spin more aggressively */
        public static final double ROTATION_SLEW = 6;

        /** The maximum allowed driving speed of the robot */
        public static final double MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        /** The maximum allowed spinning speed of the robot */
        public static final double MAX_ROTATION_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        
        /** Translation instructions closer to 0 than the deadband will be set to 0 */
        public static final double TRANSLATION_DEADBAND = .07 * MAX_TRANSLATION_SPEED;
        /** Rotation instructions closer to 0 than the deadband will be set to 0 */
        public static final double ROTATION_DEADBAND = .1 * MAX_ROTATION_SPEED;
    }

    public static final class PathPlannerConfigs {
        private static final DCMotor DRIVE_MOTOR = 
            new DCMotor(12.6, 5, 40, 20, 5, 1);
            
        private static final ModuleConfig MODULE_CONFIG = 
            new ModuleConfig(
                SwerveConstants.ModuleConstants.WHEEL_DIAMETER / 2, 
                DriveConstants.MAX_TRANSLATION_SPEED, 
                1, DRIVE_MOTOR, 60, 1
            );

        public static final RobotConfig PP_CONFIG = 
            new RobotConfig(
                40, 4, 
                MODULE_CONFIG, 
                SwerveConstants.MODULE_TRANSLATIONS
            );
    }

    /** Constants related to swerve calculations
     *  This information is less important since moving to CTRE, 
     *  but is kept for documentation purposes, and niche cases like PathPlanner configuration. */
    public static final class SwerveConstants {
        /** The distance between the left and right wheels in inches */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23);
        /** The distance between the front and rear wheels in inches */
        public static final double WHEEL_BASE = Units.inchesToMeters(23);

        /** An array containing the position of each module as a {@link Translation2d} object */
        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),  // FL
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),   // FR
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // RL
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)   // RR
        };

        /** Standard kinematics with center of rotation located at the center of the robot */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        public static final class ModuleConstants {
            /** The ratio of the drive motors on the workhorse chassis */
            public static final double DRIVE_RATIO_SLOW = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));
            /** The ratio of the drive motors on the robot */
            public static final double DRIVE_RATIO_FAST = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
            /** The ratio of the turning motors */
            public static final double TURN_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
            
            /** The diameter of the wheels measured in meters */
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.94);

            /** Drive motor revolutions * DRIVE_REVS_TO_M = distance in meters */
            public static final double DRIVE_REVS_TO_M = ((WHEEL_DIAMETER * Math.PI) / DRIVE_RATIO_FAST);

            // 1 RPM * DRIVE_REVS_TO_M = speed in m/min. Divide by 60 to find m/sec
            /** Drive motor RPM * DRIVE_RPM_TO_MPS = speed in m/sec */
            public static final double DRIVE_RPM_TO_MPS = DRIVE_REVS_TO_M / 60.0;

            /** Turning motor revolutions * TURNING_REVS_TO_DEG = Turning motor total degrees turned */
            public static final double TURNING_REVS_TO_DEG =  360.0 / TURN_RATIO;
        }
    
        /** Enum representing the four possible positions a module can occupy */
        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }
    }
}

