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

public class CoralArmIOSpark implements CoralArmIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase armLeftSpark;

  private final SparkBase armRightSpark;
  private final AbsoluteEncoder armRightEncoder;

  private final Debouncer armLeftConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer armRightConnectedDebouncer = new Debouncer(0.5);

  public CoralArmIOSpark(int armLeftCanId, int armRightCanId) {
    this.armLeftSpark = new SparkMax(armLeftCanId, SparkLowLevel.MotorType.kBrushless);

    this.armRightSpark = new SparkMax(armRightCanId, SparkLowLevel.MotorType.kBrushless);
    this.armRightEncoder = armRightSpark.getAbsoluteEncoder();

    tryUntilOk(armLeftSpark, 5, spark -> {
      spark.configure(CoralPivotConfigs.ARM_LEFT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    tryUntilOk(armRightSpark, 5, spark -> {
      spark.configure(CoralPivotConfigs.ARM_RIGHT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });
  }

  @Override public void updateInputs(CoralArmIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    useValuesIfOk(armLeftSpark,
            new DoubleSupplier[]{armLeftSpark::getAppliedOutput, armLeftSpark::getBusVoltage},
            (values) -> inputs.armLeftAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(armLeftSpark, armLeftSpark::getOutputCurrent, (value) -> inputs.armLeftCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.armLeftConnected = armLeftConnectedDebouncer.calculate(!HAS_STICKY_FAULT);

    HAS_STICKY_FAULT = false;
    useValueIfOk(armRightSpark, armRightEncoder::getPosition, (value) -> inputs.armRightPositionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(armRightSpark, armRightEncoder::getVelocity, (value) -> inputs.armRightVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(armRightSpark,
            new DoubleSupplier[]{armRightSpark::getAppliedOutput, armRightSpark::getBusVoltage},
            (values) -> inputs.armRightAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(armRightSpark, armRightSpark::getOutputCurrent, (value) -> inputs.armRightCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.armRightConnected = armRightConnectedDebouncer.calculate(!HAS_STICKY_FAULT);
  }

  @Override public void setArmPosition(double rad, double ff) {
    tryUntilOk(armRightSpark, 5, spark -> {
      spark.getClosedLoopController().setReference(
              MathUtil.clamp(rad, CoralPivotConstants.ARM_MIN_POS, CoralPivotConstants.ARM_MAX_POS),
              SparkBase.ControlType.kPosition,
              ClosedLoopSlot.kSlot0,
              ff,
              SparkClosedLoopController.ArbFFUnits.kVoltage
      );
    });
  }

  @Override public void setArmPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(armRightSpark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}
