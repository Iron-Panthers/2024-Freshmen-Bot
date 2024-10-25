// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.util.MacUtil.IS_COMP_BOT;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.Drive.Dims;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.RGBSubsystem.RGBColor;
import frc.util.CAN;
import java.nio.file.Path;
import java.util.List;

@SuppressWarnings("java:S1118")
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class Mouthy {
    public static final double SHOOT_SPEAKER_SPEED = -12;
    public static final double INTAKE_SPEED = 9;
    public static final double SHOOT_AMP_SPEED_UPPER = -1.02;
    public static final double SHOOT_AMP_SPEED_LOWER = -3;
  }

  public static final class Config {
    // maybe tune PID values?
    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(20, 0, 0),
        new PIDConstants(10, 0, 0),
        Drive.MAX_VELOCITY_METERS_PER_SECOND,
        Math.sqrt(Math.pow(Dims.TRACKWIDTH_METERS, 2) * 2),
        new ReplanningConfig());

    /** turn this off before comp. */
    public static final boolean SHOW_SHUFFLEBOARD_DEBUG_DATA = true;

    /** turn this off! only use on practice eboard testing. */
    public static final boolean DISABLE_SWERVE_INIT = false;

    /** keep this on for pigeon, disable if absolutely necessary */
    public static final boolean FLIP_GYROSCOPE = true;

    /**
     * def turn this off unless you are using it, generates in excess of 100k rows
     * for a match.
     */
    public static final boolean WRITE_APRILTAG_DATA = false;

    public static final Path APRILTAG_DATA_PATH = Filesystem.getDeployDirectory().toPath()
        .resolve("poseEstimationsAtDistances.csv");
    public static final double REAL_X = 0.0;
    public static final double REAL_Y = 0.0;
  }

  public static final class Drive {
    public static final int PIGEON_PORT = 0; // placeholder
    public static final String SWERVE_CANBUS = "rio"; // placeholder
    /** joystick xbox controller deadband **/
    public static final double DEADBAND = 0.1;
    // max voltage delivered to drivebase
    // supposedly useful to limit speed for testing
    public static final double MAX_VOLTAGE = 12.0;
    // maximum velocity
    // FIXME measure this value experimentally
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 // falcon 500 free speed rpm
        / 60.0
        * 0.10033
        * (1 / 6.12) // mk4i l3 16t falcon drive reduction (sourced from adrian)
        * Math.PI;
    // theoretical value
    // FIXME measure and validate experimentally
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
        / Math.hypot(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0)
        * .5;

    /** the maximum amount of angular error pid loops will tolerate for rotation */
    public static final double ANGULAR_ERROR = 1.0;
    /**
     * the minimum magnitude of the right stick for it to be used as a new rotation
     * angle
     */
    public static final double ROTATE_VECTOR_MAGNITUDE = .7;

    public static final class Dims {
      // FIXME validate with hardware
      public static final double TRACKWIDTH_METERS = .5207; // 20.5 inches (source: cad) converted to meters
      public static final double WHEELBASE_METERS = TRACKWIDTH_METERS; // robot is square

      public static final double BUMPER_WIDTH_METERS_X = .9779;
      public static final double BUMPER_WIDTH_METERS_Y = .8382;
    }

    /*
     * module layout:
     * |──────
     * |->│# ##steer motor
     * │ │ ##cancoder
     * │ │##drive motor
     * module number
     * 
     * steer is always left
     * from corner perspective
     * 
     * robot visualization:
     * |──────────────────────|
     * │2 10 04 1│
     * │ 25 24 │
     * │11 S D 03│
     * │ D S │
     * │ │
     * │ │
     * │ S D │
     * │ D S │
     * │12 |────────| 02│
     * │ 26 │ │ 27 │
     * │3 13│ batt │01 4│
     * |──────┴───┬┬───┴──────|
     * ││
     * ││
     * ▼▼
     * software front
     */

    public static final class Modules {
      public static final class Params {
        public static final double WHEEL_RADIUS = 2; // also in INCHES
        public static final double COUPLING_GEAR_RATIO = 3.125;
        public static final double DRIVE_GEAR_RATIO = 5.357142857142857;
        public static final double STEER_GEAR_RATIO = 21.428571428571427;
        public static final Slot0Configs DRIVE_MOTOR_GAINS = new Slot0Configs().withKP(3).withKI(0).withKD(0)
            .withKS(0.32).withKV(0.11).withKA(0);
        public static final Slot0Configs STEER_MOTOR_GAINS = new Slot0Configs().withKP(22).withKI(0).withKD(0)
            .withKS(0.4).withKV(0.6).withKA(0);
        public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
        public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
        public static final SteerFeedbackType FEEDBACK_SOURCE = SteerFeedbackType.RemoteCANcoder;
        public static final double SPEED_TWELVE_VOLTS = MAX_VELOCITY_METERS_PER_SECOND;
        public static final double SLIP_CURRENT = 0; // optional, unused rn
        public static final boolean STEER_MOTOR_INVERTED = false;

        public static final DriveRequestType driveRequestType = DriveRequestType.OpenLoopVoltage;
        public static final SteerRequestType steerRequestType = SteerRequestType.MotionMagic;
      }

      public static final class Module1 { // back left
        public static final int DRIVE_MOTOR = CAN.at(4, "module 1 drive motor");
        public static final int STEER_MOTOR = CAN.at(3, "module 1 steer motor");
        public static final int STEER_ENCODER = CAN.at(24, "module 1 steer encoder");

        public static final double STEER_OFFSET = IS_COMP_BOT
            ? 0.001708984375 // comp bot offset
            : 0.001708984375; // practice bot offset
      }

      public static final class Module2 { // back right
        public static final int DRIVE_MOTOR = CAN.at(11, "module 2 drive motor");
        public static final int STEER_MOTOR = CAN.at(10, "module 2 steer motor");
        public static final int STEER_ENCODER = CAN.at(25, "module 2 steer encoder");

        public static final double STEER_OFFSET = IS_COMP_BOT
            ? 0.366943359375 // comp bot offset
            : 0.366943359375; // practice bot offset
      }

      public static final class Module3 { // front right
        public static final int DRIVE_MOTOR = CAN.at(13, "module 3 drive motor");
        public static final int STEER_MOTOR = CAN.at(12, "module 3 steer motor");
        public static final int STEER_ENCODER = CAN.at(26, "module 3 steer encoder");

        public static final double STEER_OFFSET = IS_COMP_BOT
            ? 0.389892578125 // comp bot offset
            : 0.389892578125; // practice bot offset
      }

      public static final class Module4 { // front left
        public static final int DRIVE_MOTOR = CAN.at(2, "module 4 drive motor");
        public static final int STEER_MOTOR = CAN.at(1, "module 4 steer motor");
        public static final int STEER_ENCODER = CAN.at(27, "module 4 steer encoder");

        public static final double STEER_OFFSET = IS_COMP_BOT
            ? 0.3408203125 // comp bot offset
            : 0.3408203125; // practice bot offset
      }
    }

    public static final class Setpoints {
      public static final Translation2d SPEAKER = new Translation2d(0, 5.5);

      public static final int SOURCE_DEGREES = 39;
      public static final int SPEAKER_DEGREES = 11;
      public static final int EPSILON = 3;
    }
  }
  public static final class Amp {
      public static final class Ports {
        public static final int AMP_MOTOR_PORT = 14;
      }
      public static final double OUTTAKE_SPEED = -0.6;
      public static final double INTAKE_SPEED = 0.5;
  }


  public static final class Lights {
    public static final int CANDLE_ID = 34;
    public static final int NUM_LEDS =8; //8 in candle

    public static final class Colors {
      public static final RGBColor YELLOW = new RGBColor(255, 107, 0);
      public static final RGBColor PURPLE = new RGBColor(127, 0, 127);
      public static final RGBColor RED = new RGBColor(255, 0, 0);
      public static final RGBColor ORANGE = new RGBColor(255, 35, 0);
      public static final RGBColor BLUE = new RGBColor(0, 0, 255);
      public static final RGBColor PINK = new RGBColor(250, 35, 100);
      public static final RGBColor MINT = new RGBColor(55, 255, 50);
      public static final RGBColor TEAL = new RGBColor(0, 255, 255);
      public static final RGBColor WHITE = new RGBColor(255, 255, 255);
    }
  }
}
