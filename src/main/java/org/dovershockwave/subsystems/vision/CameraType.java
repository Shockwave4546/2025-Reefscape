package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public enum CameraType {
  LEFT_REEF_CAMERA(
          0,
          "LeftReefCamera",
          new Transform3d(0.0698, 0.260365, Units.inchesToMeters(17.125), new Rotation3d(0.0, Units.degreesToRadians(20), Units.degreesToRadians(-20))),
          1.0),

  RIGHT_REEF_CAMERA(
          0,
          "RightReefCamera",
          new Transform3d(0.0698, -0.260365, Units.inchesToMeters(17.125), new Rotation3d(0.0, Units.degreesToRadians(20), Units.degreesToRadians(20))),
          1.0),


  LEFT_HUMAN_PLAYER_STATION_CAMERA(
          1,
          "LeftHumanPlayerStationCamera",
          new Transform3d(0.0698, 0.260365, Units.inchesToMeters(31.375), new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-20))),
          1.0),

  RIGHT_HUMAN_PLAYER_STATION_CAMERA(
          1,
          "RightHumanPlayerStationCamera",
          new Transform3d(0.0698, -0.260365, Units.inchesToMeters(31.375), new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(20))),
          1.0),

  NONE(
          -1,
          "None",
          new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
          1.0);

  public final int id;
  public final String name;
  public final Transform3d robotToCamera;
  public final double stdDevFactor;

  /**
   * @param id The id of the camera
   * @param name The name of the camera
   * @param robotToCamera The transform from the robot's center to the camera's center
   * @param stdDevFactor The Standard deviation multipliers for each camera (Decrease to trust some cameras more than others)
   */
  CameraType(int id, String name, Transform3d robotToCamera, double stdDevFactor) {
    this.id = id;
    this.name = name;
    this.robotToCamera = robotToCamera;
    this.stdDevFactor = stdDevFactor;
  }
}