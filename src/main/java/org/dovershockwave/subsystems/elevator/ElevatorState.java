package org.dovershockwave.subsystems.elevator;

public record ElevatorState(String name, double positionRad) {
  public static final ElevatorState STARTING = new ElevatorState("Starting", 0.0);
  public static final ElevatorState L1 = new ElevatorState("L1", 0.0);
  public static final ElevatorState L2 = new ElevatorState("L2", 0.0);
  public static final ElevatorState L3 = new ElevatorState("L3", 0.0);
  public static final ElevatorState L4 = new ElevatorState("L4", 0.0);
  public static final ElevatorState HUMAN_PLAYER = new ElevatorState("Human Player", 0.0);
}