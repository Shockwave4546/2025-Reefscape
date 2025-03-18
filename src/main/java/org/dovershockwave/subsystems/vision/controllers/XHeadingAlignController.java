package org.dovershockwave.subsystems.vision.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.utils.PIDFGains;
import org.dovershockwave.utils.tunable.TunableProfiledPIDFController;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class XHeadingAlignController {
  private final String dashboardKey;
  private final TunableProfiledPIDFController omegaPID;
  private final TunableProfiledPIDFController xVelocityPID;
  private final DoubleSupplier yVelocitySupplier;

  public XHeadingAlignController(
          String dashboardKey,
          PIDFGains omegaPIDGains,
          PIDFGains xVelocityPIDGains,
          double headingRadTolerance,
          double xDistanceMetersTolerance,
          TrapezoidProfile.Constraints omegaConstraints,
          TrapezoidProfile.Constraints xVelocityConstraints,
          DoubleSupplier yVelocitySupplier
  ) {
    this.dashboardKey = dashboardKey;
    this.omegaPID = new TunableProfiledPIDFController(dashboardKey + "/OmegaPID", omegaPIDGains, headingRadTolerance, omegaConstraints);
    this.xVelocityPID = new TunableProfiledPIDFController(dashboardKey + "/XVelocityPID", xVelocityPIDGains, xDistanceMetersTolerance, xVelocityConstraints);
    this.yVelocitySupplier = yVelocitySupplier;

    omegaPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void resetPIDErrors(Pose2d currentPose) {
    omegaPID.reset(currentPose.getRotation().getRadians());
    xVelocityPID.reset(currentPose.getX());
  }

  public boolean atGoal() {
    return omegaPID.atGoal() && xVelocityPID.atGoal();
  }

  /**
   * @param specificDashboardKey A specific key to use for logging extra information about each calculation.
   */
  public ChassisSpeeds calculate(
          Pose2d currentPose,
          Pose2d goalPose,
          String specificDashboardKey
  ) {
    final var currentTranslationMeters = currentPose.getTranslation();
    final var omega = omegaPID.calculate(currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());
    final var xVelocity = xVelocityPID.calculate(currentTranslationMeters.getX(), goalPose.getX());

    if (RobotContainer.isTuningMode()) {
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadCurrent", currentPose.getRotation().getRadians());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadGoal", goalPose.getRotation().getRadians());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadError", omegaPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/Omega", omega);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/OmegaAtGoal", omegaPID.atGoal());

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersCurrent", currentPose.getX());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersGoal", goalPose.getX());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersError", xVelocityPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XVelocity", xVelocity);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XVelocityAtGoal", xVelocityPID.atGoal());

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/AtGoal", atGoal());
    }

    return new ChassisSpeeds(xVelocity, yVelocitySupplier.getAsDouble(), omega);
  }
}