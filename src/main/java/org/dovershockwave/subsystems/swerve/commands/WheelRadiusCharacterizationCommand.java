package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class WheelRadiusCharacterizationCommand {
  public static Command cmd(SwerveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(0.05);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                    // Reset acceleration limiter
                    Commands.runOnce(
                            () -> {
                              limiter.reset(0.0);
                            }),

                    // Turn in place, accelerating up to full speed
                    Commands.run(
                            () -> {
                              double speed = limiter.calculate(0.25);
                              drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed), false);
                            },
                            drive)),

            // Measurement sequence
            Commands.sequence(
                    // Wait for modules to fully orient before starting measurement
                    Commands.waitSeconds(1.0),

                    // Record starting measurement
                    Commands.runOnce(
                            () -> {
                              state.positions = drive.getWheelRadiusCharacterizationPositions();
                              state.lastAngle = drive.getRotation();
                              state.gyroDelta = 0.0;
                            }),

                    // Update gyro delta
                    Commands.run(
                                    () -> {
                                      var rotation = drive.getRotation();
                                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                      state.lastAngle = rotation;
                                    })

                            // When cancelled, calculate and print results
                            .finallyDo(
                                    () -> {
                                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                      double wheelDelta = 0.0;
                                      for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                      }
                                      double wheelRadius =
                                              (state.gyroDelta * SwerveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                                      NumberFormat formatter = new DecimalFormat("#0.000");
                                      System.out.println(
                                              "********** Wheel Radius Characterization Results **********");
                                      System.out.println(
                                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                      System.out.println(
                                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                      System.out.println(
                                              "\tWheel Radius: "
                                                      + formatter.format(wheelRadius)
                                                      + " meters, "
                                                      + formatter.format(Units.metersToInches(wheelRadius))
                                                      + " inches");
                                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
