package org.dovershockwave.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.PIDFGains;

public class SwerveConstants {
  public static final double ODOMETRY_FREQUENCY = 100.0; // Hz

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);
  /**
   * Distance between centers of left and right wheels
   */
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(26);
  /**
   * Distance between front and back wheels on robot
   */
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(26);
  public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(WHEEL_BASE_METERS, 2) + Math.pow(TRACK_WIDTH_METERS, 2));
  public static final double MAX_REAL_SPEED_METERS_PER_SECOND = 5.28; // TODO: 1/20/2025 - Update this value
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_REAL_SPEED_METERS_PER_SECOND / DRIVE_BASE_RADIUS;
  public static final Translation2d[] MODULE_TRANSLATIONS = {
          new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
          new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)
  };

  public static final boolean DRIVE_MOTOR_INVERTED = true;
  /**
   * MAXSwerve Extra High 2 with 14 pinion teeth and 20 spur teeth
   */
  public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 20.0) / (14.0 * 15.0);
  public static final double DRIVE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / DRIVE_MOTOR_REDUCTION;
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((2 * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0;
  public static final PIDFGains DRIVE_PIDF = new PIDFGains(0.0125, 0.0, 0.165, 0.0084);

  public static final double TURN_MOTOR_REDUCTION = 9424.0 / 203.0; // TODO: 1/24/2025 Double check this value--used for sim only 
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI;
  public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;
  public static final PIDFGains TURN_PIDF = new PIDFGains(3.1, 0.0, 0.875, 0.0);

  public static final PIDFGains DRIVE_SIM_PIDF = new PIDFGains(0.075, 0.0, 0.01, 0.1);
  public static final PIDFGains TURN_SIM_PIDF = new PIDFGains(20.0, 0.0, 0.0, 0.0);
}