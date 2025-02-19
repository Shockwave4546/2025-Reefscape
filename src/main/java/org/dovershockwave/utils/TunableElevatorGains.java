package org.dovershockwave.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dovershockwave.Constants;
import org.dovershockwave.RobotContainer;

import java.util.function.Consumer;

public class TunableElevatorGains {
  private final TunableBoolean isManualMode;
  private final TunableNumber manualValue;

  private final TunableNumber kS;
  private final TunableNumber kG;
  private final TunableNumber kV;
  private final TunableNumber kA;

  private final TunableNumber p;
  private final TunableNumber i;
  private final TunableNumber d;

  private final TunableNumber maxVelocity;
  private final TunableNumber maxAcceleration;

  private final double minValue;
  private final double maxValue;

  public TunableElevatorGains(String prefix, PIDFGains pidGains, ElevatorFeedforwardGains ffGains, TrapezoidProfile.Constraints constraints, double minValue, double maxValue) {
    isManualMode = new TunableBoolean(prefix + "(1) ManualMode", false);
    manualValue = new TunableNumber(prefix + "(2) ManualValue", 0.0, minValue, maxValue);

    kS = new TunableNumber(prefix + "(3) kS", ffGains.kS());
    kG = new TunableNumber(prefix + "(4) kG", ffGains.kG());
    kV = new TunableNumber(prefix + "(5) kV", ffGains.kV());
    kA = new TunableNumber(prefix + "(6) kA", ffGains.kA());

    p = new TunableNumber(prefix + "(7) P", pidGains.p());
    i = new TunableNumber(prefix + "(8) I", pidGains.i());
    d = new TunableNumber(prefix + "(9) D", pidGains.d());

    maxVelocity = new TunableNumber(prefix + "(10) MaxVelocity", constraints.maxVelocity);
    maxAcceleration = new TunableNumber(prefix + "(11) MaxAcceleration", constraints.maxAcceleration);

    this.minValue = minValue;
    this.maxValue = maxValue;
  }

  public TunableElevatorGains(String prefix, PIDFGains pidGains, ElevatorFeedforwardGains ffGains, TrapezoidProfile.Constraints constraints) {
    this(prefix, pidGains, ffGains, constraints, -Double.MAX_VALUE, Double.MAX_VALUE);
  }

  public void periodic(
          Consumer<PIDFGains> pidGainsConfigurator,
          Consumer<ElevatorFeedforwardGains> elevatorFeedforwardConfigurator,
          Consumer<TrapezoidProfile.Constraints> elevatorConstraintsConfigurator,
          Consumer<Double> manualValueSetter
  ) {
    if (!Constants.TUNING_MODE || RobotContainer.isCompetitionMatch()) return;

    TunableNumber.ifChanged(hashCode(), values -> pidGainsConfigurator.accept(new PIDFGains(values[0], values[1], values[2], 0.0)), p, i, d);

    TunableNumber.ifChanged(hashCode() + 1, values -> elevatorFeedforwardConfigurator.accept(
            new ElevatorFeedforwardGains(values[0], values[1], values[2], values[3])),
            kS, kG, kV, kA);

    TunableNumber.ifChanged(hashCode() + 2, values -> elevatorConstraintsConfigurator.accept(new TrapezoidProfile.Constraints(values[0], values[1])), maxVelocity, maxAcceleration);

    if (isManualMode.get()) {
      TunableNumber.ifChanged(hashCode() + 3, values -> manualValueSetter.accept(MathUtil.clamp(values[0], minValue, maxValue)), manualValue);
    }
  }

  public void setPID(PIDFGains gains) {
    p.set(gains.p());
    i.set(gains.i());
    d.set(gains.d());
  }

  public void setFFGains(ElevatorFeedforwardGains ffGains) {
    kS.set(ffGains.kS());
    kG.set(ffGains.kG());
    kV.set(ffGains.kV());
    kA.set(ffGains.kA());
  }

  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    maxVelocity.set(constraints.maxVelocity);
    maxAcceleration.set(constraints.maxAcceleration);
  }

  public PIDFGains getPIDGains() {
    return new PIDFGains(p.get(), i.get(), d.get(), 0.0);
  }

  public ElevatorFeedforwardGains getFFGains() {
    return new ElevatorFeedforwardGains(kS.get(), kG.get(), kV.get(), kA.get());
  }

  public boolean isManualMode() {
    return isManualMode.get();
  }
}