package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.VisionSubsystem;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;
import org.littletonrobotics.junction.Logger;

public class AlignToReefAlgaeCommand extends Command {
  private final FullAlignController alignController = new FullAlignController(
          "AlignToAlgaeCommand",
          VisionConstants.ALIGNMENT_OMEGA_PID,
          VisionConstants.ALIGNMENT_X_VELOCITY_PID,
          VisionConstants.ALIGNMENT_Y_VELOCITY_PID,
          VisionConstants.ALIGNMENT_RAD_TOLERANCE,
          VisionConstants.ALIGNMENT_X_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_Y_METERS_TOLERANCE,
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

    ReefScoringPosition.getAlgaePositionFor(bestTarget.tagId()).ifPresentOrElse(position -> {
      this.tagFound = true;
      final var goalPose = new Pose2d(
              position.position().toTranslation2d(),
              position.robotHeading()
      );
      final var speeds = alignController.calculate(
              swerve.getPose(),
              goalPose,
              String.valueOf(bestTarget.tagId())
      );

      Logger.recordOutput("AlignToReefAlgaeCommand/GoalPose", goalPose);
      Logger.recordOutput("AlignToReefAlgaeCommand/Vx", speeds.vxMetersPerSecond);
      Logger.recordOutput("AlignToReefAlgaeCommand/Vy", speeds.vyMetersPerSecond);
      Logger.recordOutput("AlignToReefAlgaeCommand/Omega", speeds.omegaRadiansPerSecond);

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