package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.HumanPlayerStationPosition;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;
import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.Logger;

public class AlignToHumanPlayerCommand extends Command {
  private final FullAlignController alignController = new FullAlignController(
          "AlignToHumanPlayerCommand",
          new PIDFGains(3, 0.0, 0, 0.0),
          new PIDFGains(7.5, 0.0, 0, 0.0),
          new PIDFGains(7.5, 0.0, 0, 0.0),
          Units.degreesToRadians(2),
          Units.inchesToMeters(1),
          Units.inchesToMeters(1),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED)
  );

  private final SwerveSubsystem swerve;
  private final HumanPlayerStationPosition.HumanPlayerStationSide side;

  public AlignToHumanPlayerCommand(SwerveSubsystem swerve, HumanPlayerStationPosition.HumanPlayerStationSide side) {
    this.swerve = swerve;
    this.side = side;
    addRequirements(swerve);
  }

  @Override public void initialize() {
    alignController.resetPIDErrors(swerve.getPose());
  }

  @Override public void execute() {
    HumanPlayerStationPosition.getPositionFor(swerve.getPose(), side).ifPresentOrElse(position -> {
      var speeds = alignController.calculate(
              swerve.getPose(),
              position.pose(),
              String.valueOf(position.id())
      );

      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      }

      Logger.recordOutput("AlignToHumanPlayerCommand/GoalPose", position.pose());
      Logger.recordOutput("AlignToHumanPlayerCommand/Vx", speeds.vxMetersPerSecond);
      Logger.recordOutput("AlignToHumanPlayerCommand/Vy", speeds.vyMetersPerSecond);
      Logger.recordOutput("AlignToHumanPlayerCommand/Omega", speeds.omegaRadiansPerSecond);

      swerve.runVelocityFieldRelative(speeds);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return alignController.atGoal();
  }
}