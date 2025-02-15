package org.dovershockwave.subsystems.algaepivot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.dovershockwave.Constants;

import static org.dovershockwave.subsystems.swerve.SwerveConstants.ODOMETRY_FREQUENCY;

public class AlgaePivotConfigs {
  public static final SparkBaseConfig PIVOT_CONFIG = new SparkMaxConfig();

  static {
    PIVOT_CONFIG
            .inverted(AlgaePivotConstants.DIRECTION_INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    PIVOT_CONFIG.absoluteEncoder
            .positionConversionFactor(AlgaePivotConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(AlgaePivotConstants.VELOCITY_CONVERSION_FACTOR)
            .averageDepth(2);
    PIVOT_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(AlgaePivotConstants.GAINS.p(), AlgaePivotConstants.GAINS.i(), AlgaePivotConstants.GAINS.d(), AlgaePivotConstants.GAINS.ff());
    PIVOT_CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}
