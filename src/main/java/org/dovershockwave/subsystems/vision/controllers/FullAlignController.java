package org.dovershockwave.subsystems.vision.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.utils.PIDFGains;
import org.dovershockwave.utils.tunable.TunableProfiledPIDFController;
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

    omegaPID.setTolerance(Units.degreesToRadians(2.5));
    omegaPID.enableContinuousInput(-Math.PI, Math.PI);
    xVelocityPID.setTolerance(Units.inchesToMeters(4));
    yVelocityPID.setTolerance(Units.inchesToMeters(4));

    // Used to be 1.5
  }

  public void resetPIDErrors(Pose2d currentPose) {
    omegaPID.reset(currentPose.getRotation().getRadians());
    xVelocityPID.reset(currentPose.getX());
    yVelocityPID.reset(currentPose.getY());
  }

  public boolean atGoal() {
    return omegaPID.atGoal() && xVelocityPID.atGoal() && yVelocityPID.atGoal();
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
    final var yVelocity = yVelocityPID.calculate(currentTranslationMeters.getY(), goalPose.getY());

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

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersCurrent", currentPose.getY());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersGoal", goalPose.getY());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersError", yVelocityPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YVelocity", yVelocity);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YVelocityAtGoal", yVelocityPID.atGoal());

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/AtGoal", atGoal());
    }

    return new ChassisSpeeds(xVelocity, yVelocity, omega);
  }
}