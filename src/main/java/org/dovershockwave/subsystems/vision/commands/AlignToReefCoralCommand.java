package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;
import org.dovershockwave.subsystems.vision.controllers.XHeadingAlignController;
import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.Logger;

public class AlignToReefCoralCommand extends Command {
  private final FullAlignController fullAlignController = new FullAlignController(
          "AlignToReefCoralCommand",
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

  private final XHeadingAlignController xHeadingAlignController;

  private final SwerveSubsystem swerve;
  private final ReefScoringSelector selector;
  private final ReefScoringPosition.ReefScoringSide side;

  public AlignToReefCoralCommand(SwerveSubsystem swerve, ReefScoringSelector selector, ReefScoringPosition.ReefScoringSide side, CommandXboxController controller) {
    this.swerve = swerve;
    this.selector = selector;
    this.side = side;
    this.xHeadingAlignController = new XHeadingAlignController(
            "AlignToReefCoralL1Command",
            new PIDFGains(2, 0.0, 0, 0.0),
            new PIDFGains(7.5, 0.0, 0, 0.0),
            Units.degreesToRadians(2),
            Units.inchesToMeters(1),
            new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
            new TrapezoidProfile.Constraints(2, 2),
            () -> -controller.getLeftX() * SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND * SwerveSubsystem.getVelocityMultiplier()
    );
    selector.setSide(side);
    addRequirements(swerve);
  }

  @Override public void initialize() {
    fullAlignController.resetPIDErrors(swerve.getPose());
    xHeadingAlignController.resetPIDErrors(swerve.getPose());
  }

  @Override public void execute() {
    ReefScoringPosition.getCoralPositionFor(swerve.getPose(), side, selector.getLevel()).ifPresentOrElse(position -> {
      final var goalPose = new Pose2d(
              position.position().toTranslation2d(),
              position.robotHeading()
      );
      var speeds = selector.getLevel() == ReefScoringPosition.ReefLevel.L1 ?
              xHeadingAlignController.calculate(
                      swerve.getPose(),
                      goalPose,
                      String.valueOf(position.id())
              ) :

              fullAlignController.calculate(
                      swerve.getPose(),
                      goalPose,
                      String.valueOf(position.id())
              );
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      }

      Logger.recordOutput("AlignToReefCoralCommand/GoalPose", goalPose);
      Logger.recordOutput("AlignToReefCoralCommand/Vx", speeds.vxMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralCommand/Vy", speeds.vyMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralCommand/Omega", speeds.omegaRadiansPerSecond);

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