package org.dovershockwave.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import org.dovershockwave.utils.PIDFGains;

import java.util.function.DoubleSupplier;

import static org.dovershockwave.utils.SparkUtils.*;

public class ElevatorIOSpark implements ElevatorIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase leftSpark;
  private final RelativeEncoder leftEncoder;

  private final SparkBase rightSpark;
  private final RelativeEncoder rightEncoder;

  private final DigitalInput limitSwitch = new DigitalInput(9);

  private final Debouncer leftConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer rightConnectedDebouncer = new Debouncer(0.5);

  private final Debouncer limitSwitchDebouncer = new Debouncer(0.25);

  public ElevatorIOSpark(int leftCanId, int rightCanId) {
    this.leftSpark = new SparkMax(leftCanId, SparkLowLevel.MotorType.kBrushless);
    this.leftEncoder = leftSpark.getEncoder();

    this.rightSpark = new SparkMax(rightCanId, SparkLowLevel.MotorType.kBrushless);
    this.rightEncoder = rightSpark.getEncoder();

    tryUntilOk(leftSpark, 5, spark -> {
      spark.configure(ElevatorConfigs.LEFT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    tryUntilOk(rightSpark, 5, spark -> {
      spark.configure(ElevatorConfigs.RIGHT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    resetPosition();
  }

  @Override public void updateInputs(ElevatorIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    useValueIfOk(leftSpark, leftEncoder::getPosition, (value) -> inputs.leftPositionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(leftSpark, leftEncoder::getVelocity, (value) -> inputs.leftVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(leftSpark,
            new DoubleSupplier[]{leftSpark::getAppliedOutput, leftSpark::getBusVoltage},
            (values) -> inputs.leftAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(leftSpark, leftSpark::getOutputCurrent, (value) -> inputs.leftCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.leftConnected = leftConnectedDebouncer.calculate(!HAS_STICKY_FAULT);

    HAS_STICKY_FAULT = false;
    useValueIfOk(rightSpark, rightEncoder::getPosition, (value) -> inputs.rightPositionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(rightSpark, rightEncoder::getVelocity, (value) -> inputs.rightVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(rightSpark,
            new DoubleSupplier[]{rightSpark::getAppliedOutput, rightSpark::getBusVoltage},
            (values) -> inputs.rightAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(rightSpark, rightSpark::getOutputCurrent, (value) -> inputs.rightCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.rightConnected = rightConnectedDebouncer.calculate(!HAS_STICKY_FAULT);

    inputs.limitSwitchValue = limitSwitchDebouncer.calculate(limitSwitch.get());
  }

  @Override public void setPosition(double rad, double ff) {
    tryUntilOk(leftSpark, 5, spark -> {
      spark.getClosedLoopController().setReference(
              MathUtil.clamp(rad, ElevatorConstants.MIN_POS, ElevatorConstants.MAX_POS),
              SparkBase.ControlType.kPosition,
              ClosedLoopSlot.kSlot0,
              ff,
              SparkClosedLoopController.ArbFFUnits.kVoltage
      );
    });
  }

  @Override public void resetPosition() {
    tryUntilOk(leftSpark, 5, spark -> {
      spark.getEncoder().setPosition(0);
    });

    tryUntilOk(rightSpark, 5, spark -> {
      spark.getEncoder().setPosition(0);
    });
  }

  @Override public void setPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(leftSpark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}