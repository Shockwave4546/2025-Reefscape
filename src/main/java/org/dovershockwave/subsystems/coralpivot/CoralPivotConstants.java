package org.dovershockwave.subsystems.coralpivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.ArmFeedforwardGains;
import org.dovershockwave.utils.PIDFGains;

public class CoralPivotConstants {
  public static final int WRIST_SPARK_ID = 33; // NEO 550
  public static final int ARM_LEFT_SPARK_ID = 34; // NEO
  public static final int ARM_RIGHT_SPARK_ID = 35; // NEO

  public static final boolean WRIST_INVERTED = true;
  public static final boolean ARM_PIVOT_INVERTED = true;

  public static final double WRIST_POSITION_CONVERSION_FACTOR = 2 * Math.PI;
  public static final double WRIST_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI) / 60.0;

  public static final double ARM_POSITION_CONVERSION_FACTOR = 2 * Math.PI;
  public static final double ARM_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI) / 60.0;

  public static final double WRIST_MIN_POS = -Math.PI / 2.0;
  public static final double WRIST_MAX_POS = Math.PI;

  public static final double ARM_MIN_POS = -Math.PI / 2.0;
  public static final double ARM_MAX_POS = Math.PI / 2.0;

  public static final ArmFeedforwardGains WRIST_FEEDFORWARD_GAINS = new ArmFeedforwardGains(0.0, 0.0, 3.0, 0.0);
  public static final PIDFGains WRIST_GAINS = new PIDFGains(1.5, 0.0, 0.0, 0.0);
  public static final TrapezoidProfile.Constraints WRIST_CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI);

  public static final ArmFeedforwardGains ARM_FEEDFORWARD_GAINS = new ArmFeedforwardGains(0.0, 0.16, 1.5, 0.015);
  public static final PIDFGains ARM_GAINS = new PIDFGains(0.45, 0.0, 0.125, 0.0);
  public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI);

  public static final double WRIST_POSITION_TOLERANCE = Units.degreesToRadians(4.0);
  public static final double ARM_POSITION_TOLERANCE =  Units.degreesToRadians(4.0);
}