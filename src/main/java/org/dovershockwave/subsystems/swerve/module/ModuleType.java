package org.dovershockwave.subsystems.swerve.module;

/**
 * Drive Motor Can ID Format = 1_
 * Turn Motor Can ID Format = 2_
 */
public enum ModuleType {
  FRONT_LEFT("FrontLeft", 10, 20, -Math.PI),
  FRONT_RIGHT("FrontRight", 11, 21, -Math.PI / 2),
  BACK_LEFT("BackLeft", 12, 22, Math.PI / 2),
  BACK_RIGHT("BackRight", 13, 23, 0.0);

  public final String name;
  public final int driveId;
  public final int turnId;
  public final double angleOffset;

  ModuleType(String name, int driveId, int turnId, double angleOffset) {
    this.name = name;
    this.driveId = driveId;
    this.turnId = turnId;
    this.angleOffset = angleOffset;
  }
}