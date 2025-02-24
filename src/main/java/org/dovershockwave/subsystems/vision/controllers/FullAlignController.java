package org.dovershockwave.subsystems.vision.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dovershockwave.Constants;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.utils.PIDFGains;
import org.dovershockwave.utils.TunableNumber;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

// TODO: 2/1/2025 Fix robot relative speeds
public class FullAlignController {
  private static final TrapezoidProfile.Constraints X_VELOCITY_CONSTRAINTS = new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED);
  private static final TrapezoidProfile.Constraints Y_VELOCITY_CONSTRAINTS = new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED);

  private final String dashboardKey;
  private final TunableNumber headingRadTolerance;
  private final TunableNumber xDistanceMetersTolerance;
  private final TunableNumber yDistanceMetersTolerance;

  private final TunablePIDF tunableOmegaPID;
  private final TunablePIDF tunableXVelocityPID;
  private final TunablePIDF tunableYVelocityPID;

  private final PIDController omegaPID;
  private final PIDController xVelocityPID;
  private final PIDController yVelocityPID;

  public FullAlignController(
          String dashboardKey,
          double headingRadTolerance,
          double xDistanceMetersTolerance,
          double yDistanceMetersTolerance,
          PIDFGains omegaPIDGains,
          PIDFGains xVelocityPIDGains,
          PIDFGains yVelocityPIDGains
  ) {
    this.dashboardKey = dashboardKey;
    this.headingRadTolerance = new TunableNumber(dashboardKey + "/HeadingRadTolerance", headingRadTolerance);
    this.xDistanceMetersTolerance = new TunableNumber(dashboardKey + "/XDistanceMetersTolerance", xDistanceMetersTolerance);
    this.yDistanceMetersTolerance = new TunableNumber(dashboardKey + "/YDistanceMetersTolerance", yDistanceMetersTolerance);

    this.tunableOmegaPID = new TunablePIDF(dashboardKey + "/OmegaPID", omegaPIDGains);
    this.tunableXVelocityPID = new TunablePIDF(dashboardKey + "/XVelocityPID", xVelocityPIDGains);
    this.tunableYVelocityPID = new TunablePIDF(dashboardKey + "/YVelocityPID", yVelocityPIDGains);

    this.omegaPID = new PIDController(tunableOmegaPID.getGains().p(), tunableOmegaPID.getGains().i(), tunableOmegaPID.getGains().d());
    this.xVelocityPID = new PIDController(tunableXVelocityPID.getGains().p(), tunableXVelocityPID.getGains().i(), tunableXVelocityPID.getGains().d());
    this.yVelocityPID = new PIDController(tunableYVelocityPID.getGains().p(), tunableYVelocityPID.getGains().i(), tunableYVelocityPID.getGains().d());

    omegaPID.enableContinuousInput(-Math.PI, Math.PI);
    omegaPID.setTolerance(this.headingRadTolerance.get());
    xVelocityPID.setTolerance(this.xDistanceMetersTolerance.get());
    yVelocityPID.setTolerance(this.yDistanceMetersTolerance.get());
  }

  public void resetPIDErrors() {
    omegaPID.reset();
    xVelocityPID.reset();
    yVelocityPID.reset();
  }

  /**
   * Pulls the PID gains & tolerances from the dashboard and updates the PID controllers.
   */
  public void refreshPIDControllers() {
    omegaPID.setPID(tunableOmegaPID.getGains().p(), tunableOmegaPID.getGains().i(), tunableOmegaPID.getGains().d());
    xVelocityPID.setPID(tunableXVelocityPID.getGains().p(), tunableXVelocityPID.getGains().i(), tunableXVelocityPID.getGains().d());
    yVelocityPID.setPID(tunableYVelocityPID.getGains().p(), tunableYVelocityPID.getGains().i(), tunableYVelocityPID.getGains().d());

    omegaPID.setTolerance(this.headingRadTolerance.get());
    xVelocityPID.setTolerance(this.xDistanceMetersTolerance.get());
    yVelocityPID.setTolerance(this.yDistanceMetersTolerance.get());
  }

  public boolean atSetpoint() {
    return omegaPID.atSetpoint() && xVelocityPID.atSetpoint() && yVelocityPID.atSetpoint();
  }

  /**
   * @param specificDashboardKey A specific key to use for logging extra information about each calculation.
   */
  public ChassisSpeeds calculate(
          double currentHeadingRad,
          double goalHeadingRad,
          Translation2d currentTranslationMeters,
          Translation2d targetTranslationMeters,
          String specificDashboardKey
  ) {
    final var omega = omegaPID.calculate(currentHeadingRad, goalHeadingRad);
    final var xVelocity = xVelocityPID.calculate(currentTranslationMeters.getX(), targetTranslationMeters.getX());
    final var yVelocity = yVelocityPID.calculate(currentTranslationMeters.getY(), targetTranslationMeters.getY());

    if (Constants.TUNING_MODE && !RobotContainer.isCompetitionMatch()) {
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadCurrent", currentHeadingRad);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadGoal", goalHeadingRad);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadError", omegaPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/Omega", omega);

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersCurrent", currentTranslationMeters.getX());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersGoal", targetTranslationMeters.getX());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XDistanceMetersError", xVelocityPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/XVelocity", xVelocity);

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersCurrent", currentTranslationMeters.getY());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersGoal", targetTranslationMeters.getY());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YDistanceMetersError", yVelocityPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/YVelocity", yVelocity);

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/AtSetpoint", atSetpoint());
    }
    return new ChassisSpeeds(xVelocity, yVelocity, omega);
  }
}