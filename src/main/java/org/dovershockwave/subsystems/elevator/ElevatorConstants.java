package org.dovershockwave.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.ElevatorFeedforwardGains;
import org.dovershockwave.utils.PIDFGains;

public class ElevatorConstants {
  public static final int LEFT_SPARK_ID = 37; // Neo
  public static final int RIGHT_SPARK_ID = 38; // Neo

  public static final boolean DIRECTION_INVERTED = false;

  public static final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / 6.0;
  public static final double VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / 6.0) / 60.0;

  public static final double MIN_POS = 0.0;
  public static final double MAX_POS = 48.5;

  public static final ElevatorFeedforwardGains FEEDFORWARD_GAINS = new ElevatorFeedforwardGains(0.0, 0.33, 0.0, 0.0);
  public static final PIDFGains PID_GAINS = new PIDFGains(0.4, 0.0, 0.0, 0.0);
  public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  public static final double POSITION_TOLERANCE = Units.degreesToRadians(25);
}