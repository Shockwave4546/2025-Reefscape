package org.dovershockwave.subsystems.algaerollers;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.dovershockwave.Constants;
import org.dovershockwave.subsystems.swerve.SwerveConstants;

public class AlgaeRollersConfigs {
  public static final SparkBaseConfig CONFIG = new SparkMaxConfig();

  static {
    CONFIG
            .inverted(AlgaeRollersConstants.MOTOR_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_550_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    CONFIG.encoder
            .positionConversionFactor(AlgaeRollersConstants.ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(AlgaeRollersConstants.ENCODER_VELOCITY_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(10);
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
