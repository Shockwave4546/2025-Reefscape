package org.dovershockwave.subsystems.climb;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.dovershockwave.Constants;

import static org.dovershockwave.subsystems.swerve.SwerveConstants.ODOMETRY_FREQUENCY;

public class ClimbConfigs {
  public static final SparkBaseConfig CLIMB_CONFIG = new SparkMaxConfig();

  static {
    CLIMB_CONFIG
            .inverted(ClimbConstants.DIRECTION_INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    CLIMB_CONFIG.absoluteEncoder
            .positionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR)
            .averageDepth(2);
    CLIMB_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(ClimbConstants.GAINS.p(), ClimbConstants.GAINS.i(), ClimbConstants.GAINS.d(), ClimbConstants.GAINS.ff());
    CLIMB_CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}
