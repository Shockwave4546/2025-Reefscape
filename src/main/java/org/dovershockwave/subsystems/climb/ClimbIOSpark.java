package org.dovershockwave.subsystems.climb;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import org.dovershockwave.utils.PIDFGains;

import java.util.function.DoubleSupplier;

import static org.dovershockwave.utils.SparkUtils.*;

public class ClimbIOSpark implements ClimbIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase spark;
  private final AbsoluteEncoder encoder;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ClimbIOSpark(int sparkId) {
    this.spark = new SparkMax(sparkId, SparkLowLevel.MotorType.kBrushless);
    this.encoder = spark.getAbsoluteEncoder();

    tryUntilOk(spark, 5, spark -> {
      spark.configure(ClimbConfigs.CLIMB_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });
  }

  @Override public void updateInputs(ClimbIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    useValueIfOk(spark, encoder::getPosition, (value) -> inputs.positionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(spark,
            new DoubleSupplier[]{spark::getAppliedOutput, spark::getBusVoltage},
            (values) -> inputs.appliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.connected = connectedDebouncer.calculate(!HAS_STICKY_FAULT);
  }

  @Override public void setPivotPosition(double rad) {
    tryUntilOk(spark, 5, spark -> {
      spark.getClosedLoopController().setReference(rad, SparkBase.ControlType.kPosition);
    });
  }

  @Override public void setPivotPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(spark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}
