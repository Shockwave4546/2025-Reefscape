package org.dovershockwave.utils.tunable;

import edu.wpi.first.math.MathUtil;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.utils.PIDFGains;

import java.util.function.Consumer;

public class TunablePIDF {
  protected final TunableBoolean isManualMode;
  private final TunableNumber manualValue;
  private final TunableNumber p;
  private final TunableNumber i;
  private final TunableNumber d;
  private final TunableNumber ff;

  private final double minValue;
  private final double maxValue;

  public TunablePIDF(String prefix, PIDFGains defaultGains, double minValue, double maxValue) {
    isManualMode = new TunableBoolean(prefix + "(1) ManualMode", false);
    manualValue = new TunableNumber(prefix + "(2) ManualValue", 0.0, minValue, maxValue);
    p = new TunableNumber(prefix + "(3) P", defaultGains.p());
    i = new TunableNumber(prefix + "(4) I", defaultGains.i());
    d = new TunableNumber(prefix + "(5) D", defaultGains.d());
    ff = new TunableNumber(prefix + "(6) FF", defaultGains.ff());
    this.minValue = minValue;
    this.maxValue = maxValue;
  }

  public TunablePIDF(String prefix, PIDFGains defaultGains) {
    this(prefix, defaultGains, -Double.MAX_VALUE, Double.MAX_VALUE);
  }

  public void periodic(Consumer<PIDFGains> pidfConfigurator, Consumer<Double> manualValueSetter) {
    if (!RobotContainer.isTuningMode()) return;

    TunableNumber.ifChanged(hashCode(), values -> pidfConfigurator.accept(new PIDFGains(values[0], values[1], values[2], values[3])), p, i, d, ff);

    if (isManualMode.get()) {
      TunableNumber.ifChanged(hashCode() + 1, values -> manualValueSetter.accept(MathUtil.clamp(values[0], minValue, maxValue)), manualValue);
    }
  }

  public void setGains(PIDFGains gains) {
    p.set(gains.p());
    i.set(gains.i());
    d.set(gains.d());
    ff.set(gains.ff());
  }

  public PIDFGains getGains() {
    return new PIDFGains(p.get(), i.get(), d.get(), ff.get());
  }

  public boolean isManualMode() {
    return isManualMode.get();
  }
}
