package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.VisionSubsystem;
import org.dovershockwave.subsystems.vision.controllers.HeadingController;

// TODO: 2/2/2025 Lowk could delete this, don't actually need this, but this is like boilerplate for align to human player?
public class AlignHeadingToTagCommand extends Command {
  private final HeadingController headingController = new HeadingController("AlignToTagCommand/HeadingController", VisionConstants.ALIGNMENT_RAD_TOLERANCE, VisionConstants.ALIGNMENT_OMEGA_PID);
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final CameraType camera;
  private boolean tagFound = false;

  public AlignHeadingToTagCommand(SwerveSubsystem swerve, VisionSubsystem vision, CameraType camera) {
    this.swerve = swerve;
    this.vision = vision;
    this.camera = camera;
    addRequirements(swerve, vision);
  }

  @Override public void initialize() {
    headingController.resetPIDError();
    headingController.refreshPIDController();
  }

  @Override public void execute() {
    final var bestTarget = vision.getBestTargetObservation(camera);

    ReefScoringPosition.getPositionFor(bestTarget.tagId(), ReefScoringPosition.ReefScoringSide.LEFT, ReefScoringPosition.ReefLevel.L1).ifPresentOrElse(position -> {
      this.tagFound = true;
      final var speeds = headingController.calculate(swerve.getRotation().getRadians(), position.robotHeading().getRadians(), String.valueOf(bestTarget.tagId()));
      swerve.runVelocity(speeds, false);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return tagFound && headingController.atSetpoint();
  }
}