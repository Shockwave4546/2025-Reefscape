package org.dovershockwave.subsystems.vision.controllers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.utils.PIDFGains;
import org.dovershockwave.utils.TunableProfiledPIDFController;
import org.littletonrobotics.junction.Logger;

public class HeadingController {
  private final String dashboardKey;
  private final TunableProfiledPIDFController omegaPID;

  public HeadingController(String dashboardKey, PIDFGains omegaPIDGains, double headingRadTolerance, TrapezoidProfile.Constraints constraints) {
    this.dashboardKey = dashboardKey;
    this.omegaPID = new TunableProfiledPIDFController(dashboardKey + "/OmegaPID/", omegaPIDGains, headingRadTolerance, constraints);

    omegaPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void resetPIDError(double currentHeadingRad) {
    omegaPID.reset(currentHeadingRad);
  }

  public boolean atGoal() {
    return omegaPID.atGoal();
  }

  /**
   * @param specificDashboardKey A specific key to use for logging extra information about each calculation.
   */
  public ChassisSpeeds calculate(double currentHeadingRad, double goalHeadingRad, String specificDashboardKey) {
    final var omega = omegaPID.calculate(currentHeadingRad, goalHeadingRad);

    if (RobotContainer.isTuningMode()) {
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadCurrent", currentHeadingRad);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadGoal", goalHeadingRad);
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/HeadingRadError", omegaPID.getError());
      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/Omega", omega);

      Logger.recordOutput(dashboardKey + "/" + specificDashboardKey + "/AtGoal", atGoal());
    }

    return new ChassisSpeeds(0.0, 0.0, omega);
  }
}