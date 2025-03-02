package org.dovershockwave;

import org.littletonrobotics.junction.AutoLogOutput;

public class ReefScoringSelector {
  private ReefScoringPosition.ReefScoringSide side = ReefScoringPosition.ReefScoringSide.LEFT;
  private ReefScoringPosition.ReefLevel level = ReefScoringPosition.ReefLevel.L1;

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
  }

  public void setSide(ReefScoringPosition.ReefScoringSide side) {
    this.side = side;
  }
}