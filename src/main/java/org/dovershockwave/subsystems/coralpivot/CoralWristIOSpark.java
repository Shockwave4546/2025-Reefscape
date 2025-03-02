package org.dovershockwave.subsystems.coralpivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import org.dovershockwave.utils.PIDFGains;

import java.util.function.DoubleSupplier;

import static org.dovershockwave.utils.SparkUtils.*;

public class CoralWristIOSpark implements CoralWristIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase wristSpark;
  private final AbsoluteEncoder wristEncoder;

  private final Debouncer wristConnectedDebouncer = new Debouncer(0.5);

  public CoralWristIOSpark(int wristCanId) {
    this.wristSpark = new SparkMax(wristCanId, SparkLowLevel.MotorType.kBrushless);
    this.wristEncoder = wristSpark.getAbsoluteEncoder();

    tryUntilOk(wristSpark, 5, spark -> {
      spark.configure(CoralPivotConfigs.WRIST_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });
  }

  @Override public void updateInputs(CoralWristIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    useValueIfOk(wristSpark, wristEncoder::getPosition, (value) -> inputs.wristPositionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(wristSpark, wristEncoder::getVelocity, (value) -> inputs.wristVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(wristSpark,
            new DoubleSupplier[]{wristSpark::getAppliedOutput, wristSpark::getBusVoltage},
            (values) -> inputs.wristAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(wristSpark, wristSpark::getOutputCurrent, (value) -> inputs.wristCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.wristConnected = wristConnectedDebouncer.calculate(!HAS_STICKY_FAULT);
  }

  @Override public void setWristPosition(double rad, double ff) {
    tryUntilOk(wristSpark, 5, spark -> spark.getClosedLoopController().setReference(
            MathUtil.clamp(rad, CoralPivotConstants.ARM_MIN_POS, CoralPivotConstants.ARM_MAX_POS),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ff,
            SparkClosedLoopController.ArbFFUnits.kVoltage
    ));
  }

  @Override public void setWristPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(wristSpark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}
