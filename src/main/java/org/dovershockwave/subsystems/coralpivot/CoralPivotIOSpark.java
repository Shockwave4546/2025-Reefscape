package org.dovershockwave.subsystems.coralpivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import org.dovershockwave.utils.PIDFGains;

import java.util.function.DoubleSupplier;

import static org.dovershockwave.utils.SparkUtils.*;

public class CoralPivotIOSpark implements CoralPivotIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase wristSpark;
  private final AbsoluteEncoder wristEncoder;

  private final SparkBase biggerPivotLeftSpark;
  private final AbsoluteEncoder biggerPivotLeftEncoder;

  private final SparkBase biggerPivotRightSpark;

  private final Debouncer wristConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer biggerPivotLeftConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer biggerPivotRightConnectedDebouncer = new Debouncer(0.5);

  public CoralPivotIOSpark(int wristCanId, int biggerPivotLeftCanId, int biggerPivotRightCanId) {
    this.wristSpark = new SparkMax(wristCanId, SparkLowLevel.MotorType.kBrushless);
    this.wristEncoder = wristSpark.getAbsoluteEncoder();

    this.biggerPivotLeftSpark = new SparkMax(biggerPivotLeftCanId, SparkLowLevel.MotorType.kBrushless);
    this.biggerPivotLeftEncoder = biggerPivotLeftSpark.getAbsoluteEncoder();

    this.biggerPivotRightSpark = new SparkMax(biggerPivotRightCanId, SparkLowLevel.MotorType.kBrushless);

    tryUntilOk(wristSpark, 5, spark -> {
      spark.configure(CoralPivotConfigs.WRIST_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    tryUntilOk(biggerPivotLeftSpark, 5, spark -> {
      spark.configure(CoralPivotConfigs.BIGGER_PIVOT_LEFT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    tryUntilOk(biggerPivotRightSpark, 5, spark -> {
      spark.configure(CoralPivotConfigs.BIGGER_PIVOT_RIGHT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });
  }

  @Override public void updateInputs(CoralPivotIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    useValueIfOk(wristSpark, wristEncoder::getPosition, (value) -> inputs.wristPositionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(wristSpark, wristEncoder::getVelocity, (value) -> inputs.wristVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(wristSpark,
            new DoubleSupplier[]{wristSpark::getAppliedOutput, wristSpark::getBusVoltage},
            (values) -> inputs.wristAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(wristSpark, wristSpark::getOutputCurrent, (value) -> inputs.wristCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.wristConnected = wristConnectedDebouncer.calculate(!HAS_STICKY_FAULT);

    HAS_STICKY_FAULT = false;
    useValueIfOk(biggerPivotLeftSpark, biggerPivotLeftEncoder::getPosition, (value) -> inputs.biggerPivotLeftPositionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(biggerPivotLeftSpark, biggerPivotLeftEncoder::getVelocity, (value) -> inputs.biggerPivotLeftVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(biggerPivotLeftSpark,
            new DoubleSupplier[]{biggerPivotLeftSpark::getAppliedOutput, biggerPivotLeftSpark::getBusVoltage},
            (values) -> inputs.biggerPivotLeftAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(biggerPivotLeftSpark, biggerPivotLeftSpark::getOutputCurrent, (value) -> inputs.biggerPivotLeftCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.biggerPivotLeftConnected = biggerPivotLeftConnectedDebouncer.calculate(!HAS_STICKY_FAULT);

    HAS_STICKY_FAULT = false;
    useValuesIfOk(biggerPivotRightSpark,
            new DoubleSupplier[]{biggerPivotRightSpark::getAppliedOutput, biggerPivotRightSpark::getBusVoltage},
            (values) -> inputs.biggerPivotRightAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(biggerPivotRightSpark, biggerPivotRightSpark::getOutputCurrent, (value) -> inputs.biggerPivotRightCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.biggerPivotRightConnected = biggerPivotRightConnectedDebouncer.calculate(!HAS_STICKY_FAULT);
  }

  @Override public void setWristPosition(double rad) {
    tryUntilOk(wristSpark, 5, spark -> {
      spark.getClosedLoopController().setReference(rad, SparkBase.ControlType.kPosition);
    });
  }

  @Override public void setBiggerPivotPosition(double rad, double ff) {
    tryUntilOk(biggerPivotLeftSpark, 5, spark -> {
      spark.getClosedLoopController().setReference(rad, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kVoltage);
    });
  }

  @Override public void setWristPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(wristSpark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }

  @Override public void setBiggerPivotPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(biggerPivotLeftSpark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}
