package org.dovershockwave.subsystems.elevator;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.dovershockwave.Constants;
import org.dovershockwave.subsystems.swerve.SwerveConstants;

public class ElevatorConfigs {
  public static final SparkBaseConfig LEFT_CONFIG = new SparkMaxConfig();
  public static final SparkBaseConfig RIGHT_CONFIG = new SparkMaxConfig();

  static {
    LEFT_CONFIG
            .inverted(ElevatorConstants.DIRECTION_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    LEFT_CONFIG.encoder
            .positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(10);
    LEFT_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.GAINS.p(), ElevatorConstants.GAINS.i(), ElevatorConstants.GAINS.d(), ElevatorConstants.GAINS.ff());
    LEFT_CONFIG.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConstants.ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

    RIGHT_CONFIG
            .follow(ElevatorConstants.LEFT_SPARK_ID, true)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    RIGHT_CONFIG.encoder
            .positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(10);
    RIGHT_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.GAINS.p(), ElevatorConstants.GAINS.i(), ElevatorConstants.GAINS.d(), ElevatorConstants.GAINS.ff());
    RIGHT_CONFIG.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConstants.ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}
