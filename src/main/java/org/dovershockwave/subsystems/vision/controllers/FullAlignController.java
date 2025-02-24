package org.dovershockwave.subsystems.vision.controllers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.utils.PIDFGains;
import org.dovershockwave.utils.TunableProfiledPIDFController;
import org.littletonrobotics.junction.Logger;

public class FullAlignController {
  private final String dashboardKey;
  private final TunableProfiledPIDFController omegaPID;
  private final TunableProfiledPIDFController xVelocityPID;
  private final TunableProfiledPIDFController yVelocityPID;

  public FullAlignController(
          String dashboardKey,
          PIDFGains omegaPIDGains,
          PIDFGains xVelocityPIDGains,
          PIDFGains yVelocityPIDGains,
          double headingRadTolerance,
          double xDistanceMetersTolerance,
          double yDistanceMetersTolerance,
          TrapezoidProfile.Constraints omegaConstraints,
          TrapezoidProfile.Constraints xVelocityConstraints,
          TrapezoidProfile.Constraints yVelocityConstraints
  ) {
    this.dashboardKey = dashboardKey;
    this.omegaPID = new TunableProfiledPIDFController(dashboardKey + "/OmegaPID", omegaPIDGains, headingRadTolerance, omegaConstraints);
    this.xVelocityPID = new TunableProfiledPIDFController(dashboardKey + "/XVelocityPID", xVelocityPIDGains, xDistanceMetersTolerance, xVelocityConstraints);
    this.yVelocityPID = new TunableProfiledPIDFController(dashboardKey + "/YVelocityPID", yVelocityPIDGains, yDistanceMetersTolerance, yVelocityConstraints);

    omegaPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void resetPIDErrors(double currentHeadingRad, Translation2d currentTranslationMeters) {
    omegaPID.reset(currentHeadingRad);
    xVelocityPID.reset(currentTranslationMeters.getX());
    yVelocityPID.reset(currentTranslationMeters.getY());
  }


  public boolean atGoal() {
    return omegaPID.atGoal() && xVelocityPID.atGoal() && yVelocityPID.atGoal();
  }

  /**
   * @param specificDashboardKey A specific key to use for logging extra information about each calculation.
   */
  public ChassisSpeeds calculate(
          double currentHeadingRad,
          double goalHeadingRad,
          Translation2d currentTranslationMeters,
          Translation2d goalTranslationMeters,
          String specificDashboardKey
  ) {
    final var omega = omegaPID.calculate(currentHeadingRad, goalHeadingRad);
    final var xVelocity = xVelocityPID.calculate(currentTranslationMeters.getX(), goalTranslationMeters.getX());
    final var yVelocity = yVelocityPID.calculate(currentTranslationMeters.getY(), goalTranslationMeters.getY());

    if (RobotContainer.isTuningMode()) {
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadCurrent", currentHeadingRad);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadGoal", goalHeadingRad);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadError", omegaPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/Omega", omega);

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersCurrent", currentTranslationMeters.getX());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersGoal", goalTranslationMeters.getX());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersError", xVelocityPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XVelocity", xVelocity);

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersCurrent", currentTranslationMeters.getY());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersGoal", goalTranslationMeters.getY());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersError", yVelocityPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YVelocity", yVelocity);

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/AtGoal", atGoal());
    }

    return new ChassisSpeeds(xVelocity, yVelocity, omega);
  }
}