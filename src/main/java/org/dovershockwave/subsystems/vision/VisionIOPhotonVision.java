package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.dovershockwave.utils.tunable.TunableNumber;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionIOPhotonVision implements VisionIO {
  /**
   * Increase (>1.0) if you trust your provided heading more and want to constrain it strongly.
   * Decrease (<1.0) if you want the solver to rely more on visual reprojection.
   */
  private static final TunableNumber HEADING_SCALE_FACTOR = new TunableNumber("Vision/HeadingScaleFactor", 1000000.0);
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final Supplier<Rotation2d> robotHeadingSupplier;

  public VisionIOPhotonVision(CameraType type, Supplier<Rotation2d> robotHeadingSupplier) {
    camera = new PhotonCamera(type.name);
    this.robotToCamera = type.robotToCamera;
    this.robotHeadingSupplier = robotHeadingSupplier;
    this.poseEstimator = new PhotonPoseEstimator(VisionConstants.APRIL_TAG_FIELD, PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  @Override public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    final var tagIds = new HashSet<Short>();
    final var poseObservations = new LinkedList<PoseObservation>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        // TODO: 2/18/2025 this is stupid, fix this
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

      poseEstimator.addHeadingData(result.getTimestampSeconds(), robotHeadingSupplier.get());
      poseEstimator.update(
              result,
              Optional.empty(),
              Optional.empty(),
              Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, HEADING_SCALE_FACTOR.get()))).ifPresent(poseEstimate -> {
                if (poseEstimate.targetsUsed.isEmpty()) return;
                poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(), // Timestamp
                        poseEstimate.estimatedPose, // 3D pose estimate
                        poseEstimate.targetsUsed.get(0).poseAmbiguity, // Ambiguity (This only matters for the first tag since ambiguity is only checked is tag count is 1)
                        poseEstimate.targetsUsed.size(), // Tag count
                        poseEstimate.targetsUsed.stream().mapToDouble(it -> it.getBestCameraToTarget().getTranslation().getNorm()).average().orElse(0.0))); // Average tag distance
      });

      result.targets.forEach(target -> tagIds.add((short) target.fiducialId));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
     // Save tag IDs to inputs objects
    inputs.tagIds = tagIds.stream().mapToInt(i -> i).toArray();
  }
}
