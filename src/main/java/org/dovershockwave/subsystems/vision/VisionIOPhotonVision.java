package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;

import java.util.HashSet;
import java.util.LinkedList;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public VisionIOPhotonVision(CameraType type) {
    camera = new PhotonCamera(type.name);
    this.robotToCamera = type.robotToCamera;
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

        // Get the target pose relative to the camera
        final var cameraToTarget = bestTarget.getBestCameraToTarget();

        // Invert robotToCamera to get cameraToRobot
        final var cameraToRobot = this.robotToCamera.inverse();

        // Combine cameraToTarget with cameraToRobot to find the robot pose relative to the target
        final var robotToTarget = cameraToTarget.plus(cameraToRobot);
        // TODO: 1/30/25 This needs to be resolved later bc the optional can be empty.
        final var fieldToTarget = VisionConstants.APRIL_TAG_FIELD.getTagPose(bestTarget.fiducialId).get(); // Define the field-to-target Pose3d (AprilTag position on the field)

        // Combine fieldToTarget with the inverted robotToTarget to get fieldToRobot
        final var fieldToRobot = fieldToTarget.transformBy(robotToTarget.inverse());

        tagIds.add((short) bestTarget.fiducialId);

        poseObservations.add(new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                fieldToRobot, // 3D pose estimate
                bestTarget.poseAmbiguity, // Ambiguity
                1, // Tag count
                cameraToTarget.getTranslation().getNorm())); // Average tag distance
      });
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
