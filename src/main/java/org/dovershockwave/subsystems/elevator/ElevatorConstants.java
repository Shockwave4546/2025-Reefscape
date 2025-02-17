package org.dovershockwave.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.ElevatorFeedforwardGains;
import org.dovershockwave.utils.PIDFGains;

public class ElevatorConstants {
  public static final int LEFT_SPARK_ID = 37; // Neo
  public static final int RIGHT_SPARK_ID = 38; // Neo

  public static final boolean DIRECTION_INVERTED = false;

  public static final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / 5.0;
  public static final double VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / 5.0) / 60.0;

  public static final double MIN_POS = 0.0;
  /**
   * 25:1 gearbox to a 48:24 gear reduction to a 24T pulley
   * Belt pitch of 0.005m
   * 34.455 in OR 0.875157 m for a full extension?
   */
  public static final double MAX_POS = 0.875157 * ((5 * (48.0 / 24.0) * 2.0 * Math.PI) / (24 * 0.005));

  public static final PIDFGains PID_GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0);
  public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, 10.0);
  public static final ElevatorFeedforwardGains FEEDFORWARD_GAINS = new ElevatorFeedforwardGains(0.0, 0.0, 0.0, 0.0);
  public static final double POSITION_TOLERANCE = Units.degreesToRadians(2.5);

  public static final double LIDAR_STARTING_DISTANCE = 0.0; // TODO: 1/30/25  
  public static final double LIDAR_STARTING_DISTANCE_TOLERANCE = Units.inchesToMeters(1.0); // TODO: 1/30/25  
}