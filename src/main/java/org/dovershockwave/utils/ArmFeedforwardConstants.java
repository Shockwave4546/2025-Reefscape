package org.dovershockwave.utils;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record ArmFeedforwardConstants(double kS, double kG, double kV, double kA, TrapezoidProfile.Constraints constraints) {}