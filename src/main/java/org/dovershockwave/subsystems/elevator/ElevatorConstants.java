package org.dovershockwave.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.PIDFGains;

public class ElevatorConstants {
  public static final int LEFT_SPARK_ID = 30; // Neo
  public static final int RIGHT_SPARK_ID = 31; // Neo

  public static final boolean DIRECTION_INVERTED = false;

  public static final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / 125.0;
  public static final double VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / 125.0) / 60.0;
  public static final PIDFGains GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0);
  public static final double POSITION_TOLERANCE = Units.degreesToRadians(2.5);

  public static final double LIDAR_STARTING_DISTANCE = 0.0; // TODO: 1/30/25  
  public static final double LIDAR_STARTING_DISTANCE_TOLERANCE = Units.inchesToMeters(1.0); // TODO: 1/30/25  
}