package org.dovershockwave.subsystems.coralpivot;

import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.PIDFGains;

public class CoralPivotConstants {
  public static final int WRIST_SPARK_ID = 33; // NEO 550
  public static final int BIGGER_PIVOT_LEFT_SPARK_ID = 34; // NEO
  public static final int BIGGER_PIVOT_RIGHT_SPARK_ID = 35; // NEO

  public static final boolean WRIST_INVERTED = false;
  public static final boolean BIGGER_PIVOT_INVERTED = false;

  public static final double WRIST_REDUCTION = 125.0;
  public static final double WRIST_POSITION_CONVERSION_FACTOR = (2 * Math.PI) / WRIST_REDUCTION;
  public static final double WRIST_VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / WRIST_REDUCTION) / 60.0;

  public static final double BIGGER_PIVOT_REDUCTION = 125.0;
  public static final double BIGGER_PIVOT_POSITION_CONVERSION_FACTOR = (2 * Math.PI) / BIGGER_PIVOT_REDUCTION;
  public static final double BIGGER_PIVOT_VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / BIGGER_PIVOT_REDUCTION) / 60.0;

  public static final PIDFGains WRIST_GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0);
  public static final PIDFGains BIGGER_PIVOT_GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0);
  public static final double WRIST_POSITION_TOLERANCE = Units.degreesToRadians(2.5);
  public static final double BIGGER_PIVOT_POSITION_TOLERANCE =  Units.degreesToRadians(2.5);
}