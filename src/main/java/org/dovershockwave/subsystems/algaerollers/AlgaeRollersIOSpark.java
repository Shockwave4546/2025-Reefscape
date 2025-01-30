package org.dovershockwave.subsystems.algaerollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import org.dovershockwave.utils.PIDFGains;

import java.util.function.DoubleSupplier;

import static org.dovershockwave.utils.SparkUtils.*;

public class AlgaeRollersIOSpark implements AlgaeRollersIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase spark;
  private final RelativeEncoder encoder;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public AlgaeRollersIOSpark(int sparkCanId) {
    this.spark = new SparkMax(sparkCanId, SparkLowLevel.MotorType.kBrushless);
    this.encoder = spark.getEncoder();

    tryUntilOk(spark, 5, spark -> {
      spark.configure(AlgaeRollersConfigs.CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });
  }

  @Override public void updateInputs(AlgaeRollersIOInputs inputs) {
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

  @Override public void setVelocity(double velocityRadPerSec) {
    tryUntilOk(spark, 5, spark -> {
      spark.getClosedLoopController().setReference(velocityRadPerSec, SparkBase.ControlType.kVelocity);
    });
  }

  @Override public void setPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(spark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}