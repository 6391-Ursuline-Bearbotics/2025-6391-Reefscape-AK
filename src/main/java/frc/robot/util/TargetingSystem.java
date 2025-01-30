package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.field.AllianceFlipUtil;
import frc.robot.util.field.FieldConstants.Reef;
import frc.robot.util.field.FieldConstants.ReefHeight;
import org.littletonrobotics.junction.AutoLogOutput;

public class TargetingSystem {

  private AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private ReefBranch targetBranch = ReefBranch.A;
  private ReefBranchLevel targetBranchLevel = ReefBranchLevel.L4;
  private Transform2d robotBranchScoringOffset =
      new Transform2d(Inches.of(12).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));

  public double getTargetBranchHeightMeters() {
    switch (targetBranchLevel) {
      case L1 -> {
        return ReefHeight.L1.height;
      }
      case L2 -> {
        return ReefHeight.L2.height;
      }
      case L3 -> {
        return ReefHeight.L3.height;
      }
      case L4 -> {
        return ReefHeight.L4.height;
      }
    }
    return 0;
  }

  public double getTargetBranchAlgaeArmAngle() {

    return 0;
  }

  public double getTargetBranchCoralArmAngle() {
    switch (targetBranchLevel) {
      case L1 -> {
        return ReefHeight.L1.pitch;
      }
      case L2 -> {
        return ReefHeight.L2.pitch;
      }
      case L3 -> {
        return ReefHeight.L3.pitch;
      }
      case L4 -> {
        return ReefHeight.L4.pitch;
      }
    }
    return 0;
  }

  public void setTarget(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel) {
    this.targetBranch = targetBranch;
    this.targetBranchLevel = targetBranchLevel;
  }

  public void setTargetBranch(ReefBranch targetBranch) {
    this.targetBranch = targetBranch;
  }

  public Command setTargetLevel(ReefBranchLevel targetBranchLevel) {
    return runOnce(() -> this.targetBranchLevel = targetBranchLevel);
  }

  public Command moveTargetBranchLeft() {
    return runOnce(() -> targetBranch = targetBranch.previous());
  }

  public Command moveTargetBranchRight() {
    return runOnce(() -> targetBranch = targetBranch.next());
  }

  public void left() {
    if (targetBranch == ReefBranch.H) {
      targetBranch = ReefBranch.I;
    }
  }

  public Pose2d getTargetPose() {
    Pose2d scoringPose = Pose2d.kZero;
    if (targetBranch != null)
      scoringPose =
          Reef.branchPositions
              .get(targetBranch.ordinal())
              .get(ReefHeight.L2)
              .toPose2d()
              .plus(robotBranchScoringOffset);
    return AllianceFlipUtil.apply(scoringPose);
  }

  public enum ReefBranch {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L;

    private static final ReefBranch[] vals = values();

    public ReefBranch next() {
      return vals[(this.ordinal() + 1) % vals.length];
    }

    public ReefBranch previous() {
      // Move backward one ordinal, adding vals.length before modulo to avoid negative index
      return vals[(this.ordinal() - 1 + vals.length) % vals.length];
    }
  }

  public enum ReefBranchLevel {
    L1,
    L2,
    L3,
    L4
  }

  @AutoLogOutput(key = "TargetLevel")
  public String getLevel() {
    return targetBranchLevel.toString();
  }

  @AutoLogOutput(key = "TargetBranch")
  public String getBranch() {
    return targetBranch.toString();
  }
}
