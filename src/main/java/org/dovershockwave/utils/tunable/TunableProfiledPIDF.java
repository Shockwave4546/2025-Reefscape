package org.dovershockwave.utils.tunable;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.utils.PIDFGains;

import java.util.function.Consumer;

public class TunableProfiledPIDF extends TunablePIDF {
  private final TunableNumber maxVelocity;
  private final TunableNumber maxAcceleration;

  public TunableProfiledPIDF(String prefix, PIDFGains defaultGains, TrapezoidProfile.Constraints constraints, double minValue, double maxValue) {
    super(prefix, defaultGains, minValue, maxValue);

    maxVelocity = new TunableNumber(prefix + "(7) MaxVelocity", constraints.maxVelocity);
    maxAcceleration = new TunableNumber(prefix + "(8) MaxAcceleration", constraints.maxAcceleration);
  }

  public TunableProfiledPIDF(String prefix, PIDFGains defaultGains, TrapezoidProfile.Constraints constraints) {
    this(prefix, defaultGains, constraints, -Double.MAX_VALUE, Double.MAX_VALUE);
  }

  public void periodic(Consumer<PIDFGains> pidfConfigurator, Consumer<TrapezoidProfile.Constraints> constraintsConfigurator, Consumer<Double> manualValueSetter) {
    super.periodic(pidfConfigurator, manualValueSetter);

    if (RobotContainer.isTuningMode())
      TunableNumber.ifChanged(hashCode() + 2, values -> constraintsConfigurator.accept(new TrapezoidProfile.Constraints(values[0], values[1])), maxVelocity, maxAcceleration);
  }

  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    maxVelocity.set(constraints.maxVelocity);
    maxAcceleration.set(constraints.maxAcceleration);
  }
}
