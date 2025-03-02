package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

import java.text.DecimalFormat;

public class WheelRadiusCharacterizationCommand extends ParallelCommandGroup {
  private final SlewRateLimiter limiter = new SlewRateLimiter(0.05);
  private final WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

  public WheelRadiusCharacterizationCommand(SwerveSubsystem swerve) {
    addCommands(
            new SequentialCommandGroup(
                    new InstantCommand(() -> limiter.reset(0.0)),
                    new RunCommand(() -> {
                      final var speed = limiter.calculate(0.25);
                      swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, speed), false);
                    }, swerve)
            ),
            new SequentialCommandGroup(
                    // Wait for modules to fully orient before starting measurement
                    new WaitCommand(1.0),
                    new InstantCommand(() -> {
                      state.positions = swerve.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = swerve.getRotation();
                      state.gyroDelta = 0.0;
                    }),
                    new RunCommand(() -> {
                      final var rotation = swerve.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    }).finallyDo(() -> {  // When cancelled, calculate and print results
                      final var positions = swerve.getWheelRadiusCharacterizationPositions();
                      var wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }

                      final var wheelRadius = (state.gyroDelta * SwerveConstants.DRIVE_BASE_RADIUS) / wheelDelta;
                      final var formatter = new DecimalFormat("#0.00000");
                      System.out.println("********** Wheel Radius Characterization Results **********");
                      System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, " + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
                    })
            )
    );

    addRequirements(swerve);
  }

  private static class WheelRadiusCharacterizationState {
    public double[] positions = new double[4];
    public Rotation2d lastAngle = new Rotation2d();
    public double gyroDelta = 0.0;
  }
}