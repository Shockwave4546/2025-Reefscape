package org.dovershockwave.subsystems.vision.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dovershockwave.Constants;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.utils.PIDFGains;
import org.dovershockwave.utils.TunableNumber;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

// TODO: 2/1/2025 Fix robot relative speeds
public class HeadingController {
  private final String dashboardKey;
  private final TunableNumber headingRadTolerance;
  private final TunablePIDF tunableOmegaPID;
  private final PIDController omegaPID;

  public HeadingController(String dashboardKey, double headingRadTolerance, PIDFGains omegaPIDGains) {
    this.dashboardKey = dashboardKey;
    this.headingRadTolerance = new TunableNumber(dashboardKey + "/HeadingRadTolerance", headingRadTolerance);
    this.tunableOmegaPID = new TunablePIDF(dashboardKey + "/OmegaPID", omegaPIDGains);
    this.omegaPID = new PIDController(tunableOmegaPID.getGains().p(), tunableOmegaPID.getGains().i(), tunableOmegaPID.getGains().d());
    omegaPID.enableContinuousInput(-Math.PI, Math.PI);
    omegaPID.setTolerance(this.headingRadTolerance.get());
  }

  public void resetPIDError() {
    omegaPID.reset();
  }

  /**
   * Pulls the PID gains & tolerance from the dashboard and updates the PID controller.
   */
  public void refreshPIDController() {
    omegaPID.setPID(tunableOmegaPID.getGains().p(), tunableOmegaPID.getGains().i(), tunableOmegaPID.getGains().d());
    omegaPID.setTolerance(headingRadTolerance.get());
  }

  public boolean atSetpoint() {
    return omegaPID.atSetpoint();
  }

  /**
   * @param specificDashboardKey A specific key to use for logging extra information about each calculation.
   */
  public ChassisSpeeds calculate(double currentHeadingRad, double goalHeadingRad, String specificDashboardKey) {
    final var omega = omegaPID.calculate(currentHeadingRad, goalHeadingRad);

    if (Constants.TUNING_MODE && !RobotContainer.isCompetitionMatch()) {
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadCurrent", currentHeadingRad);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadGoal", goalHeadingRad);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadError", omegaPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/Omega", omega);

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/AtSetpoint", atSetpoint());
    }

    return new ChassisSpeeds(0.0, 0.0, omega);
  }
}