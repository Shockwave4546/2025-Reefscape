package org.dovershockwave.subsystems.elevator;

public record ElevatorState(double positionRad) {
  public static final ElevatorState STARTING = new ElevatorState(0.0);
  public static final ElevatorState L1 = new ElevatorState(0.0);
  public static final ElevatorState L2 = new ElevatorState(0.0);
  public static final ElevatorState L3 = new ElevatorState(41.0);
  public static final ElevatorState L4 = new ElevatorState(97.25);
  public static final ElevatorState HUMAN_PLAYER = new ElevatorState(0.0);

  public static final ElevatorState KNOCKOFF_ALGAE_L3 = new ElevatorState(10);
  public static final ElevatorState KNOCKOFF_ALGAE_L2 = new ElevatorState(0);
}