package org.dovershockwave.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.ElevatorFeedforwardConstants;
import org.dovershockwave.utils.PIDFGains;

public class ElevatorConstants {
  public static final int LEFT_SPARK_ID = 37; // Neo
  public static final int RIGHT_SPARK_ID = 38; // Neo

  public static final boolean DIRECTION_INVERTED = false;

  public static final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / 125.0;
  public static final double VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / 125.0) / 60.0;
  public static final PIDFGains PID_GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0);
  public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, 10.0);
  public static final ElevatorFeedforwardConstants FEEDFORWARD_CONSTANTS = new ElevatorFeedforwardConstants(0.0, 0.0, 0.0, 0.0);
  public static final double POSITION_TOLERANCE = Units.degreesToRadians(2.5);

  public static final double LIDAR_STARTING_DISTANCE = 0.0; // TODO: 1/30/25  
  public static final double LIDAR_STARTING_DISTANCE_TOLERANCE = Units.inchesToMeters(1.0); // TODO: 1/30/25  
}