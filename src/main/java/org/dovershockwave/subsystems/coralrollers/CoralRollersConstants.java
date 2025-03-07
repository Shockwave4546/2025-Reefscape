package org.dovershockwave.subsystems.coralrollers;

public class CoralRollersConstants {
  public static final int SPARK_ID = 36; // NEO 550

  public static final boolean MOTOR_INVERTED = false;

  public static final double MOTOR_REDUCTION = 100.0;
  public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI) / MOTOR_REDUCTION;
  public static final double ENCODER_VELOCITY_FACTOR = ((2 * Math.PI) / MOTOR_REDUCTION) / 60.0;

  /**
   * If the Lidar is reporting a measurement less than this number, we can assume a coral is in position.
   */
  public static final double CORAL_IN_POSITION_LIDAR_DISTANCE_MIN = 0.02;
}