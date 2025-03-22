package org.dovershockwave.auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;
import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.Logger;

public class AlignToReefCoralL4Command extends Command {
  private final FullAlignController fullAlignController = new FullAlignController(
          "AlignToReefCoralL4Command",
          new PIDFGains(2, 0.0, 0, 0.0),
          new PIDFGains(7.5, 0.0, 0, 0.0),
          new PIDFGains(7.5, 0.0, 0, 0.0),
          Units.degreesToRadians(2),
          Units.inchesToMeters(1),
          Units.inchesToMeters(1),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
          new TrapezoidProfile.Constraints(2.5, 2),
          new TrapezoidProfile.Constraints(2.5, 2)
  );

  private final SwerveSubsystem swerve;
  private final ReefScoringSelector selector;
  private final ReefScoringPosition.ReefScoringSide side;

  public AlignToReefCoralL4Command(SwerveSubsystem swerve, ReefScoringSelector selector, ReefScoringPosition.ReefScoringSide side) {
    this.swerve = swerve;
    this.selector = selector;
    this.side = side;
    selector.setSide(side);
    selector.setLevel(ReefScoringPosition.ReefLevel.L4);
    addRequirements(swerve);
  }

  @Override public void initialize() {
    fullAlignController.resetPIDErrors(swerve.getPose());
  }

  @Override public void execute() {
    ReefScoringPosition.getCoralPositionFor(swerve.getPose(), side, selector.getLevel()).ifPresentOrElse(position -> {
      final var goalPose = new Pose2d(
              position.position().toTranslation2d(),
              position.robotHeading()
      );
      var speeds = fullAlignController.calculate(
              swerve.getPose(),
              goalPose,
              String.valueOf(position.id())
      );
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      }

      Logger.recordOutput("AlignToReefCoralL4Command/GoalPose", goalPose);
      Logger.recordOutput("AlignToReefCoralL4Command/Vx", speeds.vxMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralL4Command/Vy", speeds.vyMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralL4Command/Omega", speeds.omegaRadiansPerSecond);

      swerve.runVelocityFieldRelative(speeds);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return fullAlignController.atGoal();
  }
}