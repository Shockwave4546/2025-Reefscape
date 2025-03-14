package org.dovershockwave;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.subsystems.swerve.SwerveConstants;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;

public record ReefScoringPosition(int id, Translation3d position, Rotation2d robotHeading) {
  /**
   * Reef scoring positions with blue alliance wall as reference and robot facing the field.
   * Based off <a href="https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape-andymark.json">AndyMark field AprilTag locations</a>.
   */
  private static final Map<Integer, Pose2d> REEF_SCORING_POSE_2D = new HashMap<>();

  static {

    /*
    Blue alliance
     */
    REEF_SCORING_POSE_2D.put(18, new Pose2d(3.6576, 4.0259, Rotation2d.fromRadians(0.0)));
    REEF_SCORING_POSE_2D.put(17, new Pose2d(4.073905999999999, 3.3012379999999997, Rotation2d.fromRadians(Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(22, new Pose2d(4.904739999999999, 3.3063179999999996, Rotation2d.fromRadians(2 * Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(21, new Pose2d(5.321046, 4.0259, Rotation2d.fromRadians(Math.PI)));
    REEF_SCORING_POSE_2D.put(20, new Pose2d(4.904739999999999, 4.745482, Rotation2d.fromRadians(-2 * Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(19, new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromRadians(-Math.PI / 3)));

    /*
    Red Alliance
     */
    REEF_SCORING_POSE_2D.put(7, new Pose2d(13.890498, 4.0208200000000005, Rotation2d.fromRadians(Math.PI)));
    REEF_SCORING_POSE_2D.put(8, new Pose2d(13.474446, 4.740402, Rotation2d.fromRadians(-2 * Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(9, new Pose2d(12.643358, 4.740402, Rotation2d.fromRadians(-Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(10, new Pose2d(12.227305999999999, 4.0208200000000005, Rotation2d.fromRadians(0.0)));
    REEF_SCORING_POSE_2D.put(11, new Pose2d(12.643358, 3.3012379999999997, Rotation2d.fromRadians(Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(6, new Pose2d(13.474446, 3.3012379999999997, Rotation2d.fromRadians(2 * Math.PI / 3)));
  }

  public static OptionalDouble getRobotHeading(int id) {
    return OptionalDouble.of(REEF_SCORING_POSE_2D.get(id).getRotation().getRadians());
  }

  public static Optional<ReefScoringPosition> getCoralPositionFor(int id, ReefScoringSide side, ReefLevel level) {
    if (!REEF_SCORING_POSE_2D.containsKey(id)) return Optional.empty();

    final var additionalXOffset = switch (level) {
      case L1 -> 0.45;
      // Used to be 0.075
      case L2, L3 -> 0.1;
      case L4 -> 0.0;
    };
    final var xOffset = (-SwerveConstants.ROBOT_LENGTH_X_METERS / 2.0) - additionalXOffset;
    final var offsetPose2d = REEF_SCORING_POSE_2D.get(id).transformBy(new Transform2d(xOffset, side.yOffset, new Rotation2d()));
    return Optional.of(new ReefScoringPosition(
            id,
            new Translation3d(offsetPose2d.getX(), offsetPose2d.getY(), level.height),
            offsetPose2d.getRotation()
    ));
  }

  public static Optional<ReefScoringPosition> getCoralPositionFor(Pose2d robotPose, ReefScoringSide side, ReefLevel level) {
    return REEF_SCORING_POSE_2D.keySet().stream().min((id1, id2) -> {
      final var pose1 = REEF_SCORING_POSE_2D.get(id1);
      final var pose2 = REEF_SCORING_POSE_2D.get(id2);
      return Double.compare(
              robotPose.getTranslation().getDistance(pose1.getTranslation()),
              robotPose.getTranslation().getDistance(pose2.getTranslation())
      );
    }).flatMap(closestId -> getCoralPositionFor(closestId, side, level)); // Return empty if no closest ID is found
  }

  public static Optional<ReefScoringPosition> getAlgaePositionFor(int id) {
    if (!REEF_SCORING_POSE_2D.containsKey(id)) return Optional.empty();

    final var offsetPose2d = REEF_SCORING_POSE_2D.get(id).transformBy(new Transform2d(-SwerveConstants.ROBOT_LENGTH_X_METERS / 2.0, 0.0, new Rotation2d()));
    return Optional.of(new ReefScoringPosition(
            id,
            new Translation3d(offsetPose2d.getX(), offsetPose2d.getY(), 0.0),
            offsetPose2d.getRotation()
    ));
  }

  public static Optional<ReefScoringPosition> getAlgaePositionFor(Pose2d robotPose) {
    return REEF_SCORING_POSE_2D.keySet().stream().min((id1, id2) -> {
      final var pose1 = REEF_SCORING_POSE_2D.get(id1);
      final var pose2 = REEF_SCORING_POSE_2D.get(id2);
      return Double.compare(
              robotPose.getTranslation().getDistance(pose1.getTranslation()),
              robotPose.getTranslation().getDistance(pose2.getTranslation())
      );
    }).flatMap(ReefScoringPosition::getAlgaePositionFor); // Return empty if no closest ID is found
  }

  public enum ReefScoringSide {
    LEFT(Units.inchesToMeters(6.469)),
    RIGHT(Units.inchesToMeters(-7.5));

    public final double yOffset;

    ReefScoringSide(double yOffset) {
      this.yOffset = yOffset;
    }
  }

  public enum ReefLevel {
    L4(Units.inchesToMeters(72), Units.degreesToRadians(-90)),
    L3(Units.inchesToMeters(47.625), Units.degreesToRadians(-35)),
    L2(Units.inchesToMeters(31.875), Units.degreesToRadians(-35)),
    L1(Units.inchesToMeters(18), Units.degreesToRadians(0));

    public final double height;
    public final double pitch;

    ReefLevel(double height, double pitch) {
      this.height = height;
      this.pitch = pitch;
    }
  }
}