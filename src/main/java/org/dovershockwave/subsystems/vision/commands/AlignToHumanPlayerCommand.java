package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.VisionSubsystem;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;

// TODO: 2/2/25 Decide on cameras for this still
public class AlignToHumanPlayerCommand extends Command {
  private final FullAlignController alignController = new FullAlignController(
          "AlignToHumanPlayerCommand",
          VisionConstants.ALIGNMENT_RAD_TOLERANCE,
          VisionConstants.ALIGNMENT_X_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_Y_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_OMEGA_PID,
          VisionConstants.ALIGNMENT_X_VELOCITY_PID,
          VisionConstants.ALIGNMENT_Y_VELOCITY_PID
  );

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private boolean tagFound = false;

  public AlignToHumanPlayerCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve, vision);
  }

  @Override public void initialize() {
    alignController.resetPIDErrors();
    alignController.refreshPIDControllers();
  }

  @Override public void execute() {
    final var bestTarget = vision.getBestTargetObservation(CameraType.HUMAN_PLAYER_STATION_CAMERA);
    if (!bestTarget.hasObservation()) {
      swerve.stop();
      return;
    }

    ReefScoringPosition.getRobotHeading(bestTarget.tagId()).ifPresentOrElse(desiredHeading -> {
      this.tagFound = true;

      // TODO: 2/2/25 Fix this xOffset
      final var offsetTranslationGoal = new Translation2d(0.5, 0.0);
      final var speeds = alignController.calculate(
              swerve.getRotation().getRadians(),
              desiredHeading,
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