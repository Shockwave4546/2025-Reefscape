package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.subsystems.vision.VisionSubsystem;

/**
 * Align to an AprilTag on the Reef for L1 with a temporary heading, so that driving can be more seamless.
 */
public class TemporaryHeadingCommand extends Command {
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private Rotation2d previousHeading;
  private boolean tagFound = false;

  public TemporaryHeadingCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve, vision);
  }

  @Override public void initialize() {
    previousHeading = swerve.getRotation();
  }

  @Override public void execute() {
    /*
    If the tag has already been found, don't bother looking for it again. The robot heading
    should already be set.
     */
    if (tagFound) return;

    /*
    Default to using the left camera to align to the reef, but use the right camera if the left camera doesn't have an observation.
    If neither camera has an observation, default to null and keeps looking for a tag.
     */
    final var camera = vision.getBestTargetObservation(CameraType.LEFT_REEF_CAMERA).hasObservation() ? CameraType.LEFT_REEF_CAMERA :
            vision.getBestTargetObservation(CameraType.RIGHT_REEF_CAMERA).hasObservation() ? CameraType.RIGHT_REEF_CAMERA : null;

    if (camera == null) return;

    final var bestTarget = vision.getBestTargetObservation(camera);
    ReefScoringPosition.getRobotHeading(bestTarget.tagId()).ifPresent(desiredHeading -> {
      this.tagFound = true;
      swerve.setPose(new Pose2d(swerve.getPose().getTranslation(), Rotation2d.fromRadians(desiredHeading)));
    });
  }

  @Override public void end(boolean interrupted) {
    swerve.setPose(new Pose2d(swerve.getPose().getTranslation(), previousHeading));
  }
}