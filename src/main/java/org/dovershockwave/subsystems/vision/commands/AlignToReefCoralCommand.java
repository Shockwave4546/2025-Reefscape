package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.VisionSubsystem;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;

public class AlignToReefCoralCommand extends Command {
  private final FullAlignController alignController = new FullAlignController(
          "AlignToReefCommand",
          VisionConstants.ALIGNMENT_RAD_TOLERANCE,
          VisionConstants.ALIGNMENT_X_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_Y_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_OMEGA_PID,
          VisionConstants.ALIGNMENT_X_VELOCITY_PID,
          VisionConstants.ALIGNMENT_Y_VELOCITY_PID
  );

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final ReefScoringSelector selector;
  private boolean tagFound = false;

  public AlignToReefCoralCommand(SwerveSubsystem swerve, VisionSubsystem vision, ReefScoringSelector selector) {
    this.swerve = swerve;
    this.vision = vision;
    this.selector = selector;
    addRequirements(swerve, vision);
  }

  @Override public void initialize() {
    alignController.resetPIDErrors();
    alignController.refreshPIDControllers();
  }

  @Override public void execute() {
    selector.setSide(ReefScoringPosition.ReefScoringSide.RIGHT);
    final var camera = selector.getSide() == ReefScoringPosition.ReefScoringSide.LEFT ? CameraType.RIGHT_REEF_CAMERA : CameraType.LEFT_REEF_CAMERA;
    final var bestTarget = vision.getBestTargetObservation(camera);

    ReefScoringPosition.getPositionFor(bestTarget.tagId(), selector.getSide(), selector.getLevel()).ifPresentOrElse(position -> {
      this.tagFound = true;
      // TODO: 2/1/2025 Fix this xOffset
      final var offsetTranslationGoal = new Translation2d(0.5, selector.getSide().yOffset);
      final var speeds = alignController.calculate(
              swerve.getRotation().getRadians(),
              position.robotHeading().getRadians(),
              bestTarget.translation().toTranslation2d(),
              offsetTranslationGoal,
              String.valueOf(bestTarget.tagId())
      );

      swerve.runVelocity(speeds, false);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return tagFound && alignController.atSetpoint();
  }
}