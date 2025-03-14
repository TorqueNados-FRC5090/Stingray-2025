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

    public static final class SubsystemIDs {
        public static final int SERVO_HUB_CAN_ID = 3;
        public static final int RIO_SERVO_PORT = 2;
        public static final int CANDLE_ID = 13;

        public static final int CLIMBER_MOTOR_ID = 12;

        public static final int ELEVATOR_LEFT_MOTOR_ID = 15;
        public static final int ELEVATOR_RIGHT_MOTOR_ID = 16;
        public static final int PIVOT_MOTOR_ID = 11;

        public static final int ALGAE_LEFT_MOTOR_ID = 17;
        public static final int ALGAE_RIGHT_MOTOR_ID = 18;

        public static final int SHOOTER_ENTRY_SENSOR_ID = 20;
        public static final int SHOOTER_EXIT_SENSOR_ID = 21;
        public static final int SHOOTER_MOTOR_ID = 10;
    }

    /* -------------- SUBSYTEM CONSTANTS -------------- */

    public static final class ShooterConstants {
        public static final double P_GAIN = 0;
        /** Converts motor revolutions to inches of linear travel */
        public static final double SHOOTER_RATIO = 2 * Math.PI;
        /** The distance between the two sensors in inches */
        public static final double SENSOR_SEPARATION = 6.5; 
        /** The ideal position of the coral measured as inches 
         * from the front of the piece to the entry sensor */
        public static final double IDEAL_CORAL_POSITION = 10;
    }

    public static final class AlgaeConstants {
        public static final double P_GAIN = .015;

        public static final double ALGAE_RATIO = 360.0/125.0;

        public enum AlgaePosition {
            ZERO(0),
            OUT(75);

            private double setpoint;
            AlgaePosition(double setpoint) {
                this.setpoint = setpoint;
            };

            public double getAngle() {
                return setpoint;
            }
        }
    }


    public static final class ClimberConstants {
        public static final double P_GAIN = .27;

        /** Converts climber motor revolutions to degrees of climber travel */
        public static final double CLIMBER_RATIO = 360.0/900.0;

        public enum ClimberPosition {
            /** Vertical */
            ZERO(0),
            /** Out of robot, used to line up with cage */
            PREPARE(80),
            /** Inside robot, used when engaged with cage */
            CLIMB(-110);

            private double setpoint;
            ClimberPosition(double setpoint) {
                this.setpoint = setpoint;
            };

            /** @return The angle of the climber associated with the setpoint */
            public double getAngle() {
                return setpoint;
            }
        }
    }

    public static final class ElevatorConstants {
        public static final double P_GAIN = .225;
        public static final double D_GAIN = .005;
        public static final double VEL_LIMIT = 100;
        public static final double ACCEL_LIMIT = 59;
        
        /** Converts elevator motor revolutions to inches of shooter travel */
        public static final double ELEVATOR_RATIO = 1 / (25.4 * (1 / 19.189168));
    }

    public static final class PivotConstants {
        public static final double P_GAIN = .225;
        public static final double D_GAIN = .005;
    }

    public enum UpperChassisPose {
        ZERO(0, 0),
        TROUGH(3, 0),
        L2(9.4, 20),
        L3(25.1, 20),
        L4( 55.4, 75);
        
        private double height;
        private double angle;
        UpperChassisPose(double height, double angle) {
            this.height = height;
            this.angle = angle;
        };

        public double getHeight() {
            return height;
        }
        public double getAngle() {
            return angle;
        }
    }

    public static final class LEDConstants {
        public static enum LEDColor {
            RED(255, 0, 0),
            GREEN(25, 255, 0),
            BLUE(0, 10, 181),
            YELLOW(255, 100, 0),
            PURPLE(162, 18, 184),
            PINK(255, 166, 238),
            LIGHT_BLUE(125, 212, 255),
            ORANGE(180, 20, 0),
            WHITE(255, 255, 255);

            private int red;
            private int green;
            private int blue;
             
            LEDColor(int red, int green, int blue) {
                this.red = red;
                this.green = green;
                this.blue = blue;
            }

            public int getRed() { return red; }
            public int getGreen() { return green; }
            public int getBlue() { return blue; }
        }

        public static enum LEDStrip {
              CANDLE(0, 8),
              SHOOTER(8, 60),
              INTAKE(68, 31);

              private int startingIndex;
              private int stripLength;

              LEDStrip(int startingIndex, int stripLength){
                  this.startingIndex = startingIndex;
                  this.stripLength = stripLength;
              }

              public int getStartingIndex() { return startingIndex; }
              public int getStripLength() { return stripLength; }
        }
    }
    
    
    /* -------------- DRIVETRAIN CONSTANTS -------------- */

    public static final class DriveConstants {
        /** Translation instructions closer to 0 than the deadband will be set to 0 */
        public static final double TRANSLATION_DEADBAND = .05;
        /** Rotation instructions closer to 0 than the deadband will be set to 0 */
        public static final double ROTATION_DEADBAND = .05;

        /** Higher values make the robot drive more aggressively */
        public static final double TRANSLATION_SLEW = 4;
        /** Higher values make the robot spin more aggressively */
        public static final double ROTATION_SLEW = 6;

        /** The maximum allowed driving speed of the robot */
        public static final double MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        /** The maximum allowed spinning speed of the robot */
        public static final double MAX_ROTATION_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
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

