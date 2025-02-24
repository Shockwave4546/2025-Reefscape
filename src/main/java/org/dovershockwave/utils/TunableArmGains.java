package org.dovershockwave.utils;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dovershockwave.RobotContainer;

import java.util.function.Consumer;

public class TunableArmGains extends TunableProfiledPIDF {
  private final TunableNumber kS;
  private final TunableNumber kG;
  private final TunableNumber kV;
  private final TunableNumber kA;

  public TunableArmGains(String prefix, PIDFGains pidGains, ArmFeedforwardGains ffGains, TrapezoidProfile.Constraints constraints, double minValue, double maxValue) {
    super(prefix, pidGains, constraints, minValue, maxValue);

    kS = new TunableNumber(prefix + "(9) kS", ffGains.kS());
    kG = new TunableNumber(prefix + "(10) kG", ffGains.kG());
    kV = new TunableNumber(prefix + "(11) kV", ffGains.kV());
    kA = new TunableNumber(prefix + "(12) kA", ffGains.kA());
  }

  public TunableArmGains(String prefix, PIDFGains pidGains, ArmFeedforwardGains ffGains, TrapezoidProfile.Constraints constraints) {
    this(prefix, pidGains, ffGains, constraints, -Double.MAX_VALUE, Double.MAX_VALUE);
  }

  public void periodic(
          Consumer<PIDFGains> pidGainsConfigurator,
          Consumer<ArmFeedforwardGains> armFeedforwardConfigurator,
          Consumer<TrapezoidProfile.Constraints> armConstraintsConfigurator,
          Consumer<Double> manualValueSetter
  ) {
    super.periodic(pidGainsConfigurator, armConstraintsConfigurator, manualValueSetter);

    if (RobotContainer.isTuningMode())
      TunableNumber.ifChanged(hashCode() + 3, values -> armFeedforwardConfigurator.accept(
            new ArmFeedforwardGains(values[0], values[1], values[2], values[3])),
            kS, kG, kV, kA
      );
  }

  public void setFFGains(ArmFeedforwardGains ffGains) {
    kS.set(ffGains.kS());
    kG.set(ffGains.kG());
    kV.set(ffGains.kV());
    kA.set(ffGains.kA());
  }

  public ArmFeedforwardGains getFFGains() {
    return new ArmFeedforwardGains(kS.get(), kG.get(), kV.get(), kA.get());
  }
}