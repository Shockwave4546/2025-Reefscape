package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;
import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.Logger;

public class AlignToReefAlgaeCommand extends Command {
  private final FullAlignController alignController = new FullAlignController(
          "AlignToReefAlgaeCommand",
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

  public AlignToReefAlgaeCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override public void initialize() {
    alignController.resetPIDErrors(swerve.getPose());
  }

  @Override public void execute() {
    ReefScoringPosition.getAlgaePositionFor(swerve.getPose()).ifPresentOrElse(position -> {
      final var goalPose = new Pose2d(
              position.position().toTranslation2d(),
              position.robotHeading()
      );
      var speeds = alignController.calculate(
              swerve.getPose(),
              goalPose,
              String.valueOf(position.id())
      );
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      }

      Logger.recordOutput("AlignToReefAlgaeCommand/GoalPose", goalPose);
      Logger.recordOutput("AlignToReefAlgaeCommand/Vx", speeds.vxMetersPerSecond);
      Logger.recordOutput("AlignToReefAlgaeCommand/Vy", speeds.vyMetersPerSecond);
      Logger.recordOutput("AlignToReefAlgaeCommand/Omega", speeds.omegaRadiansPerSecond);

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