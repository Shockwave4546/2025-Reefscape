package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.VisionSubsystem;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;

public class AlignToReefAlgaeCommand extends Command {
  private final FullAlignController alignController = new FullAlignController(
          "AlignToAlgaeCommand",
          VisionConstants.ALIGNMENT_RAD_TOLERANCE,
          VisionConstants.ALIGNMENT_X_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_Y_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_OMEGA_PID,
          VisionConstants.ALIGNMENT_X_VELOCITY_PID,
          VisionConstants.ALIGNMENT_Y_VELOCITY_PID,
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED)
  );

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private boolean tagFound = false;

  public AlignToReefAlgaeCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve, vision);
  }

  @Override public void initialize() {
    // TODO: 2/24/2025 fact check this line
    alignController.resetPIDErrors(swerve.getRotation().getRadians(), swerve.getPose().getTranslation());
  }

  @Override public void execute() {
    final var camera = vision.getBestTargetObservation(CameraType.LEFT_REEF_CAMERA).hasObservation() ? CameraType.LEFT_REEF_CAMERA : CameraType.RIGHT_REEF_CAMERA;
    final var bestTarget = vision.getBestTargetObservation(camera);
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
    return tagFound && alignController.atGoal();
  }
}