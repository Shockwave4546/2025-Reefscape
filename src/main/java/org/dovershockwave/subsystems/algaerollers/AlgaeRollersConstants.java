package org.dovershockwave.subsystems.algaerollers;

import org.dovershockwave.utils.PIDFGains;

public class AlgaeRollersConstants {
  public static final int SPARK_ID = 36; // NEO 550

  public static final boolean MOTOR_INVERTED = false;

  public static final double MOTOR_REDUCTION = 125.0;
  public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI) / MOTOR_REDUCTION;
  public static final double ENCODER_VELOCITY_FACTOR = ((2 * Math.PI) / MOTOR_REDUCTION) / 60.0;

  public static final PIDFGains GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0);
}