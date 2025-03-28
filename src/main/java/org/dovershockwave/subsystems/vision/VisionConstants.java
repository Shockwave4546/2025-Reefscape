package org.dovershockwave.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.dovershockwave.utils.PIDFGains;

public class VisionConstants {
  public static final AprilTagFieldLayout APRIL_TAG_FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final double SINGLE_TAG_MAX_AMBIGUITY = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

  /**
   * Standard deviation baselines, for 1-meter distance and 1 tag (Adjusted automatically based on distance and # of tags)
   */
  public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

  public static final PIDFGains ALIGNMENT_OMEGA_PID = new PIDFGains(2.91, 0.0, 0, 0.0);
  public static final PIDFGains ALIGNMENT_X_VELOCITY_PID = new PIDFGains(7.5, 0.0, 0, 0.0);
  public static final PIDFGains ALIGNMENT_Y_VELOCITY_PID = new PIDFGains(7.5, 0.0, 0, 0.0);
}