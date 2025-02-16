package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final Supplier<Rotation2d> robotHeadingSupplier;

  public VisionIOPhotonVision(CameraType type, Supplier<Rotation2d> robotHeadingSupplier) {
    camera = new PhotonCamera(type.name);
    this.robotToCamera = type.robotToCamera;
    this.robotHeadingSupplier = robotHeadingSupplier;
  }

  @Override public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    final var tagIds = new HashSet<Short>();
    final var poseObservations = new LinkedList<PoseObservation>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        final var bestTarget = result.getBestTarget();
        final var bestTargetPose = bestTarget.getBestCameraToTarget();
        inputs.bestTargetObservation = new TargetObservation(
                true,
                bestTarget.getFiducialId(),
                Rotation2d.fromDegrees(bestTarget.getYaw()),
                Rotation2d.fromDegrees(bestTarget.getPitch()),
                bestTargetPose.getTranslation(),
                bestTargetPose.getTranslation().getNorm());

        inputs.latestTargetObservations = result.targets.stream().map(target -> {
          final var targetPose = target.getBestCameraToTarget();
          return new TargetObservation(
                  true,
                  target.getFiducialId(),
                  Rotation2d.fromDegrees(target.getYaw()),
                  Rotation2d.fromDegrees(target.getPitch()),
                  targetPose.getTranslation(),
                  targetPose.getTranslation().getNorm());
        }).toArray(TargetObservation[]::new);
      } else {
        inputs.bestTargetObservation = new TargetObservation(false, Integer.MIN_VALUE, new Rotation2d(), new Rotation2d(), new Translation3d(), 0.0);
        inputs.latestTargetObservations = new TargetObservation[0];
      }

      // Add pose observation
      result.multitagResult.ifPresentOrElse(multitagResult -> {
        // Calculate robot pose
        final var fieldToCamera = multitagResult.estimatedPose.best;
        final var fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        final var robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size())); // Average tag distance
      }, () -> {
        if (!result.hasTargets()) return;
        if (!VisionConstants.DO_SINGLE_TAG_POSE_ESTIMATE) return;
        final var bestTarget = result.getBestTarget();
        final var tagDistance = bestTarget.getBestCameraToTarget().getTranslation().getNorm();
        pnpDistanceTrigSolve(bestTarget).ifPresent(robotPose3d -> {
          tagIds.add((short) bestTarget.fiducialId);
          poseObservations.add(new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose3d, // 3D pose estimate
                  bestTarget.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  tagDistance)); // Average tag distance
                }
        );
      });
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
     // Save tag IDs to inputs objects
    inputs.tagIds = tagIds.stream().mapToInt(i -> i).toArray();
  }

  /**
   * Use distance data from the best visible tag to compute a Pose. This runs on the RoboRIO in order
   * to access the robot's yaw heading.
   *
   * <p>6328 Explanation on Chief Delphi<a href="https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/98">...</a></p>
   */
  private Optional<Pose3d> pnpDistanceTrigSolve(PhotonTrackedTarget target) {
    final var camToTagTranslation = new Translation3d(
            target.getBestCameraToTarget().getTranslation().getNorm(),
            new Rotation3d(
                    0,
                    -Math.toRadians(target.getPitch()),
                    -Math.toRadians(target.getYaw())))
            .rotateBy(robotToCamera.getRotation())
            .toTranslation2d()
            .rotateBy(robotHeadingSupplier.get());

    return VisionConstants.APRIL_TAG_FIELD.getTagPose(target.getFiducialId()).map(tagPose3d -> {
      final var fieldToCameraTranslation = tagPose3d.toPose2d().getTranslation().plus(camToTagTranslation.unaryMinus());
      final var camToRobotTranslation = robotToCamera.getTranslation().toTranslation2d().unaryMinus().rotateBy(robotHeadingSupplier.get());
      final var robotPose2d = new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), robotHeadingSupplier.get());
      return new Pose3d(robotPose2d);
    });
  }
}
