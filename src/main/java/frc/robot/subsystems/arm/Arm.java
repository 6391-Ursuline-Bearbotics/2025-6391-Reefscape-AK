package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.Util;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

@Setter
@Getter
public class Arm extends GenericMotionProfiledSubsystem<Arm.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {
    HOME(Units.degreesToRotations(20.0), 0.0, ProfileType.MM_POSITION),
    // HOMING(0.0, 0.0, ProfileType.MM_POSITION),
    INTAKE(Units.degreesToRotations(0.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_1(Units.degreesToRotations(20.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_2(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_3(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_4(Units.degreesToRotations(60.0), 0.0, ProfileType.MM_POSITION),
    GROUND(Units.degreesToRotations(105.0), 0.0, ProfileType.MM_POSITION);

    private final double output;
    private final double feedFwd;

    private final ProfileType profileType;
  }

  @Getter @Setter private State state = State.HOME;

  private final boolean debug = true;
  private Supplier<Pose3d> carriagePoseSupplier;

  public Arm(ArmIO io, boolean isSim, Supplier<Pose3d> carriagePoseSupplier) {
    super(ProfileType.MM_POSITION, ArmConstants.kSubSysConstants, io, isSim);
    this.carriagePoseSupplier = carriagePoseSupplier;
  }

  /** Constructor */
  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }

  public boolean atPosition(double tolerance) {
    return Util.epsilonEquals(io.getPosition(), state.output, Math.max(0.0001, tolerance));
  }

  @AutoLogOutput(key = "Mech2D/ArmComponentPose")
  public Pose3d getArmComponentPose() {
    return carriagePoseSupplier
        .get()
        .plus(new Transform3d(0.083, 0, 0, new Rotation3d()))
        .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -io.getPosition(), 0)));
  }

  @AutoLogOutput(key = "Mech2D/ClawComponentPose")
  public Pose3d getClawComponentPose() {
    return getArmComponentPose().plus(new Transform3d(0.2585, 0, 0, new Rotation3d()));
  }
}
