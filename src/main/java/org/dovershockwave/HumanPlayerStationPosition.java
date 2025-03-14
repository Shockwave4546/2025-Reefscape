package org.dovershockwave;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.subsystems.swerve.SwerveConstants;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public record HumanPlayerStationPosition(Pose2d pose, int id) {
  /**
   * Groove distance: 1.25"
   * Higher up plate distance: 6.75"
   * From the middle of the AprilTag: (1.25 / 2) + (2 * 6.75) + (1.25 / 2) + 1.25 = 16"
   * 0.93 / 2 = 0.465" from the center of the robot to the edge of the robot, so
   * we need to make sure the robot is 0.465" away from the wall.
   */
  private static final double CLOSE_Y_OFFSET_DISTANCE_METERS = Units.inchesToMeters(16);
  private static final double FAR_Y_OFFSET_DISTANCE_METERS = Units.inchesToMeters(16);
  private static final Map<Integer, Pose2d> HUMAN_PLAYER_STATION_POSE_2D = new HashMap<>();

  static {
    HUMAN_PLAYER_STATION_POSE_2D.put(12, new Pose2d(0.8613139999999999, 0.628142, Rotation2d.fromDegrees(-126.0)));
    HUMAN_PLAYER_STATION_POSE_2D.put(13, new Pose2d(0.8613139999999999, 7.414259999999999, Rotation2d.fromDegrees(126.0)));
    HUMAN_PLAYER_STATION_POSE_2D.put(1, new Pose2d(16.687292, 0.628142, Rotation2d.fromDegrees(-54.0)));
    HUMAN_PLAYER_STATION_POSE_2D.put(2, new Pose2d(16.687292, 7.414259999999999, Rotation2d.fromDegrees(54.0)));
  }

  public static Optional<HumanPlayerStationPosition> getPositionFor(int id, HumanPlayerStationSide side) {
    if (!HUMAN_PLAYER_STATION_POSE_2D.containsKey(id)) return Optional.empty();

    final var signForYOffset = switch (side) {
      case CLOSE -> switch (id) {
        case 1, 13 -> 1;
        case 2, 12 -> -1;
        default -> 0;
      };
      case CENTER -> 1;
      case FAR -> switch (id) {
        case 1, 13 -> -1;
        case 2, 12 -> 1;
        default -> 0;
      };
    };

    if (signForYOffset == 0) return Optional.empty();

    final var offsetXDistance = (-SwerveConstants.ROBOT_LENGTH_X_METERS / 2.0) - Units.inchesToMeters(15);
    final var offsetYDistance = switch (side) {
      case CLOSE -> CLOSE_Y_OFFSET_DISTANCE_METERS;
      case FAR -> FAR_Y_OFFSET_DISTANCE_METERS;
      case CENTER -> 0;
    };

    return Optional.of(new HumanPlayerStationPosition(HUMAN_PLAYER_STATION_POSE_2D.get(id).transformBy(
            new Transform2d(offsetXDistance, offsetYDistance * signForYOffset, new Rotation2d())),
            id
    ));
  }

  public static Optional<HumanPlayerStationPosition> getPositionFor(Pose2d robotPose, HumanPlayerStationSide side) {
    return HUMAN_PLAYER_STATION_POSE_2D.keySet().stream().min((id1, id2) -> {
      final var pose1 = HUMAN_PLAYER_STATION_POSE_2D.get(id1);
      final var pose2 = HUMAN_PLAYER_STATION_POSE_2D.get(id2);
      return Double.compare(
              robotPose.getTranslation().getDistance(pose1.getTranslation()),
              robotPose.getTranslation().getDistance(pose2.getTranslation())
      );
    }).flatMap(closestId -> getPositionFor(closestId, side)); // Return empty if no closest ID is found
  }

  public enum HumanPlayerStationSide {
    CLOSE, CENTER, FAR
  }
}