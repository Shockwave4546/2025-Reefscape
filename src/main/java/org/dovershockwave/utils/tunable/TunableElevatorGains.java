package org.dovershockwave.utils.tunable;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.utils.ElevatorFeedforwardGains;
import org.dovershockwave.utils.PIDFGains;

import java.util.function.Consumer;

public class TunableElevatorGains extends TunableProfiledPIDF {
  private final TunableNumber kS;
  private final TunableNumber kG;
  private final TunableNumber kV;
  private final TunableNumber kA;

  public TunableElevatorGains(String prefix, PIDFGains pidGains, ElevatorFeedforwardGains ffGains, TrapezoidProfile.Constraints constraints, double minValue, double maxValue) {
    super(prefix, pidGains, constraints, minValue, maxValue);

    kS = new TunableNumber(prefix + "(9) kS", ffGains.kS());
    kG = new TunableNumber(prefix + "(10) kG", ffGains.kG());
    kV = new TunableNumber(prefix + "(11) kV", ffGains.kV());
    kA = new TunableNumber(prefix + "(12) kA", ffGains.kA());
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
    super.periodic(pidGainsConfigurator, elevatorConstraintsConfigurator, manualValueSetter);

    if (RobotContainer.isTuningMode())
      TunableNumber.ifChanged(hashCode() + 3, values -> elevatorFeedforwardConfigurator.accept(
            new ElevatorFeedforwardGains(values[0], values[1], values[2], values[3])),
            kS, kG, kV, kA
      );
  }

  public void setFFGains(ElevatorFeedforwardGains ffGains) {
    kS.set(ffGains.kS());
    kG.set(ffGains.kG());
    kV.set(ffGains.kV());
    kA.set(ffGains.kA());
  }

  public ElevatorFeedforwardGains getFFGains() {
    return new ElevatorFeedforwardGains(kS.get(), kG.get(), kV.get(), kA.get());
  }
}