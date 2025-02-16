package org.dovershockwave.subsystems.coralpivot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.dovershockwave.Constants;

import static org.dovershockwave.subsystems.swerve.SwerveConstants.ODOMETRY_FREQUENCY;

public class CoralPivotConfigs {
  public static final SparkBaseConfig WRIST_CONFIG = new SparkMaxConfig();
  public static final SparkBaseConfig ARM_LEFT_CONFIG = new SparkMaxConfig();
  public static final SparkBaseConfig ARM_RIGHT_CONFIG = new SparkMaxConfig();

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

    ARM_RIGHT_CONFIG
            .inverted(CoralPivotConstants.ARM_PIVOT_INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    ARM_RIGHT_CONFIG.absoluteEncoder
            .inverted(true)
            .zeroCentered(true)
            .positionConversionFactor(CoralPivotConstants.ARM_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(CoralPivotConstants.ARM_VELOCITY_CONVERSION_FACTOR)
            .averageDepth(2);
    ARM_RIGHT_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(CoralPivotConstants.ARM_GAINS.p(), CoralPivotConstants.ARM_GAINS.i(), CoralPivotConstants.ARM_GAINS.d(), CoralPivotConstants.ARM_GAINS.ff());
    ARM_RIGHT_CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

    ARM_LEFT_CONFIG
            .follow(CoralPivotConstants.ARM_RIGHT_SPARK_ID, true)
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
  }
}