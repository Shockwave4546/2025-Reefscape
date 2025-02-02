package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public record HumanPlayerStationPosition(double yOffsetMeters, double robotHeadingRad) {
  private static final double CLOSE_Y_OFFSET_DISTANCE_METERS = 1;
  private static final double FAR_Y_OFFSET_DISTANCE_METERS = 1;
  private static final Map<Integer, Rotation2d> HUMAN_PLAYER_STATION_ROBOT_HEADING = new HashMap<>();

  static {
    // TODO: 2/2/25 My brain doesn't work, so idk what the heading is. 
    HUMAN_PLAYER_STATION_ROBOT_HEADING.put(12, new Rotation2d());
    HUMAN_PLAYER_STATION_ROBOT_HEADING.put(13, new Rotation2d());
    HUMAN_PLAYER_STATION_ROBOT_HEADING.put(1, new Rotation2d());
    HUMAN_PLAYER_STATION_ROBOT_HEADING.put(2, new Rotation2d());
  }

  public static Optional<HumanPlayerStationPosition> getPositionFor(int id, HumanPlayerStationSide side) {
    final var signForOffset = switch (side) {
      case CLOSE -> switch (id) {
        case 1, 13 -> -1;
        case 2, 12 -> 1;
        default -> 0;
      };
      case CENTER -> 1;
      case FAR -> switch (id) {
        case 1, 13 -> 1;
        case 2, 12 -> -1;
        default -> 0;
      };
    };

    if (signForOffset == 0) return Optional.empty();

    final var offsetDistance = switch (side) {
      case CLOSE -> CLOSE_Y_OFFSET_DISTANCE_METERS;
      case FAR -> FAR_Y_OFFSET_DISTANCE_METERS;
      case CENTER -> 0;
    };


    return Optional.of(new HumanPlayerStationPosition(
            signForOffset * offsetDistance,
            HUMAN_PLAYER_STATION_ROBOT_HEADING.get(id).getRadians())
    );
  }

  public enum HumanPlayerStationSide {
    CLOSE, CENTER, FAR
  }
}