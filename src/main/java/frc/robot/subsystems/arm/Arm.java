package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.util.TargetingSystem.ArmState;
import frc.robot.util.Util;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

@Setter
@Getter
public class Arm extends GenericMotionProfiledSubsystem<ArmState> {

  @Getter @Setter private ArmState state = ArmState.HOME;

  private final boolean debug = true;
  private Supplier<Pose3d> carriagePoseSupplier;

  public Arm(ArmIO io, boolean isSim, Supplier<Pose3d> carriagePoseSupplier) {
    super(ProfileType.MM_POSITION, ArmConstants.kSubSysConstants, io, isSim);
    this.carriagePoseSupplier = carriagePoseSupplier;
  }

  /** Constructor */
  public Command setStateCommand(Supplier<ArmState> state) {
    return startEnd(() -> this.state = state.get(), () -> this.state = ArmState.HOME);
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
