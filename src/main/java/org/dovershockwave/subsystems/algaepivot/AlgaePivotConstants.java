package org.dovershockwave.subsystems.algaepivot;

import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.PIDFGains;

public class AlgaePivotConstants {
  public static final int SPARK_ID = 30; // Neo

  public static final boolean DIRECTION_INVERTED = false;

  public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
  public static final double VELOCITY_CONVERSION_FACTOR = (2 * Math.PI) / 60.0;
  public static final PIDFGains GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0);
  public static final double POSITION_TOLERANCE = Units.degreesToRadians(2.5);
}
