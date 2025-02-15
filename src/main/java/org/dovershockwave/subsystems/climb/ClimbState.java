package org.dovershockwave.subsystems.climb;

public record ClimbState(double positionRad) {
  public static final ClimbState STARTING = new ClimbState(0.0);
  public static final ClimbState CLIMBING = new ClimbState(0.0);
}
