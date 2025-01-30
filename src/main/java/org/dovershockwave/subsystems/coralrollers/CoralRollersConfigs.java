package org.dovershockwave.subsystems.coralrollers;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.dovershockwave.Constants;
import org.dovershockwave.subsystems.swerve.SwerveConstants;

public class CoralRollersConfigs {
  public static final SparkBaseConfig CONFIG = new SparkMaxConfig();

  static {
    CONFIG
            .inverted(CoralRollersConstants.MOTOR_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.DRIVE_NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    CONFIG.encoder
            .positionConversionFactor(CoralRollersConstants.ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(CoralRollersConstants.ENCODER_VELOCITY_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(10);
    CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(CoralRollersConstants.GAINS.p(), CoralRollersConstants.GAINS.i(), CoralRollersConstants.GAINS.d(), CoralRollersConstants.GAINS.ff());
    CONFIG.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConstants.ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}
