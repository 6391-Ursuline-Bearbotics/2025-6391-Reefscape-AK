package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.ProfileType;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.field.AllianceFlipUtil;
import frc.robot.util.field.FieldConstants.Reef;
import frc.robot.util.field.FieldConstants.ReefHeight;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;

public class TargetingSystem {

  private AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private ReefBranch targetBranch = ReefBranch.A;
  private ElevatorState targetBranchLevel = ElevatorState.LEVEL_1;
  private Transform2d robotBranchScoringOffset =
      new Transform2d(Inches.of(12).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));

  public ElevatorState getElevatorState() {
    return targetBranchLevel;
  }

  public ArmState getArmState() {
    switch (targetBranchLevel) {
      case LEVEL_1 -> {
        return ArmState.LEVEL_1;
      }
      case LEVEL_2 -> {
        return ArmState.LEVEL_2;
      }
      case LEVEL_3 -> {
        return ArmState.LEVEL_3;
      }
      case LEVEL_4 -> {
        return ArmState.LEVEL_4;
      }
    }
    return ArmState.HOME;
  }

  public double getTargetBranchAlgaeArmAngle() {

    return 0;
  }

  public void setTarget(ReefBranch targetBranch, ElevatorState targetBranchLevel) {
    this.targetBranch = targetBranch;
    this.targetBranchLevel = targetBranchLevel;
  }

  public void setTargetBranch(ReefBranch targetBranch) {
    this.targetBranch = targetBranch;
  }

  public Command setTargetLevel(ElevatorState targetBranchLevel) {
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

  @RequiredArgsConstructor
  @Getter
  public enum ElevatorState implements TargetState {
    HOMING(
        -0.2, 0.0, ProfileType.OPEN_VOLTAGE), // TODO: Test Voltage and position values (rotations)
    // (elevator)
    HOME(0.305, 0.0, ProfileType.MM_POSITION),
    INTAKE(0.05, 0.0, ProfileType.MM_POSITION),
    LEVEL_1(0.45, 0.0, ProfileType.MM_POSITION),
    LEVEL_2(1.2, 0.0, ProfileType.MM_POSITION),
    LEVEL_3(1.4, 0.0, ProfileType.MM_POSITION),
    LEVEL_4(1.82, 0.0, ProfileType.MM_POSITION),
    CORAL_STATION(0.6, 0.0, ProfileType.MM_POSITION),
    ALGAE_LOWER(0.5, 0.0, ProfileType.MM_POSITION),
    ALGAE_UPPER(0.8, 0.0, ProfileType.MM_POSITION),
    NET(9.0, 0.0, ProfileType.MM_POSITION);

    public final double output;
    public final double feedFwd;
    public final ProfileType profileType;
  }

  @RequiredArgsConstructor
  @Getter
  public enum ArmState implements TargetState {
    HOME(Units.degreesToRotations(20.0), 0.0, ProfileType.MM_POSITION),
    // HOMING(0.0, 0.0, ProfileType.MM_POSITION),
    INTAKE(Units.degreesToRotations(0.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_1(Units.degreesToRotations(20.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_2(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_3(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_4(Units.degreesToRotations(60.0), 0.0, ProfileType.MM_POSITION),
    GROUND(Units.degreesToRotations(105.0), 0.0, ProfileType.MM_POSITION);

    public final double output;
    public final double feedFwd;

    public final ProfileType profileType;
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
