package org.dovershockwave;

import org.dovershockwave.utils.Tab;
import org.dovershockwave.utils.shuffleboard.ShuffleboardBoolean;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.EnumMap;

public class ReefScoringSelector {
   private final EnumMap<ReefScoringPosition.ReefLevel, ShuffleboardBoolean> reefLevelIndicators = new EnumMap<>(ReefScoringPosition.ReefLevel.class);
  private ReefScoringPosition.ReefScoringSide side = ReefScoringPosition.ReefScoringSide.LEFT;
  private ReefScoringPosition.ReefLevel level = ReefScoringPosition.ReefLevel.L1;

  public ReefScoringSelector() {
    for (final var level : ReefScoringPosition.ReefLevel.values()) {
      reefLevelIndicators.put(level, new ShuffleboardBoolean(Tab.MATCH, level.name(), false));
    }

    setLevel(ReefScoringPosition.ReefLevel.L1);
  }

  @AutoLogOutput(key = "Reef/ScoringLevel")
  public ReefScoringPosition.ReefLevel getLevel() {
    return level;
  }

  @AutoLogOutput(key = "Reef/ScoringSide")
  public ReefScoringPosition.ReefScoringSide getSide() {
    return side;
  }

  public void setLevel(ReefScoringPosition.ReefLevel level) {
    this.level = level;

     for (final var entry : reefLevelIndicators.entrySet()) {
       entry.getValue().set(entry.getKey() == level);
     }
  }

  public void setSide(ReefScoringPosition.ReefScoringSide side) {
    this.side = side;
  }
}