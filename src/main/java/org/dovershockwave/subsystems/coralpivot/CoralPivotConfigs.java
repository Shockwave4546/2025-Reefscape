package org.dovershockwave.subsystems.coralpivot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.dovershockwave.Constants;

import static org.dovershockwave.subsystems.swerve.SwerveConstants.ODOMETRY_FREQUENCY;

public class CoralPivotConfigs {
  public static final SparkBaseConfig WRIST_CONFIG = new SparkMaxConfig();
  public static final SparkBaseConfig BIGGER_PIVOT_LEFT_CONFIG = new SparkMaxConfig();
  public static final SparkBaseConfig BIGGER_PIVOT_RIGHT_CONFIG = new SparkMaxConfig();

  static {
    WRIST_CONFIG
            .inverted(CoralPivotConstants.WRIST_INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_550_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    WRIST_CONFIG.absoluteEncoder
            .zeroCentered(true)
            .positionConversionFactor(CoralPivotConstants.WRIST_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(CoralPivotConstants.WRIST_VELOCITY_CONVERSION_FACTOR)
            .averageDepth(2);
    WRIST_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(CoralPivotConstants.WRIST_GAINS.p(), CoralPivotConstants.WRIST_GAINS.i(), CoralPivotConstants.WRIST_GAINS.d(),CoralPivotConstants.WRIST_GAINS.ff());
    WRIST_CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

    BIGGER_PIVOT_LEFT_CONFIG
            .inverted(CoralPivotConstants.BIGGER_PIVOT_INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    BIGGER_PIVOT_LEFT_CONFIG.absoluteEncoder
            .positionConversionFactor(CoralPivotConstants.BIGGER_PIVOT_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(CoralPivotConstants.BIGGER_PIVOT_VELOCITY_CONVERSION_FACTOR)
            .averageDepth(2);
    BIGGER_PIVOT_LEFT_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(CoralPivotConstants.BIGGER_PIVOT_GAINS.p(), CoralPivotConstants.BIGGER_PIVOT_GAINS.i(), CoralPivotConstants.BIGGER_PIVOT_GAINS.d(), CoralPivotConstants.BIGGER_PIVOT_GAINS.ff());
    BIGGER_PIVOT_LEFT_CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

    BIGGER_PIVOT_RIGHT_CONFIG
            .follow(CoralPivotConstants.BIGGER_PIVOT_LEFT_SPARK_ID)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
  }
}