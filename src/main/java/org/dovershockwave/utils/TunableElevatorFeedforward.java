package org.dovershockwave.utils;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.function.Consumer;

public class TunableElevatorFeedforward {
  private final TunableBoolean isManualMode;
  private final TunableNumber manualValue;
  private final TunableNumber kS;
  private final TunableNumber kG;
  private final TunableNumber kV;
  private final TunableNumber kA;

  private final TunableNumber maxVelocity;
  private final TunableNumber maxAcceleration;

  public TunableElevatorFeedforward(String prefix, ElevatorFeedforwardConstants constants) {
    isManualMode = new TunableBoolean(prefix + "(1) ManualMode", false);
    manualValue = new TunableNumber(prefix + "(2) ManualValue", 0.0);
    kS = new TunableNumber(prefix + "(3) kS", constants.kS());
    kG = new TunableNumber(prefix + "(4) kG", constants.kG());
    kV = new TunableNumber(prefix + "(5) kV", constants.kV());
    kA = new TunableNumber(prefix + "(6) kA", constants.kA());
    maxVelocity = new TunableNumber(prefix + "(7) MaxVelocity", constants.constraints().maxVelocity);
    maxAcceleration = new TunableNumber(prefix + "(8) MaxAcceleration", constants.constraints().maxAcceleration);
  }

  public void periodic(Consumer<ElevatorFeedforwardConstants> elevatorFeedforwardConfigurator, Consumer<Double> manualValueSetter) {
    TunableNumber.ifChanged(hashCode(), values -> elevatorFeedforwardConfigurator.accept(
            new ElevatorFeedforwardConstants(values[0], values[1], values[2], values[3], new TrapezoidProfile.Constraints(values[4], values[5]))),
            kS, kG, kV, kA, maxVelocity, maxAcceleration);

    if (isManualMode.get()) {
      TunableNumber.ifChanged(hashCode() * 2, values -> manualValueSetter.accept(values[0]), manualValue);
    }
  }

  public void setConstants(ElevatorFeedforwardConstants constants) {
    kS.set(constants.kS());
    kG.set(constants.kG());
    kV.set(constants.kV());
    kA.set(constants.kA());
    maxVelocity.set(constants.constraints().maxVelocity);
    maxAcceleration.set(constants.constraints().maxAcceleration);
  }

  public ElevatorFeedforwardConstants getConstants() {
    return new ElevatorFeedforwardConstants(kS.get(), kG.get(), kV.get(), kA.get(), new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
  }

  public boolean isManualMode() {
    return isManualMode.get();
  }
}