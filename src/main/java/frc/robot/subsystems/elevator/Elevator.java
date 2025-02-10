package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.util.TargetingSystem.ElevatorState;
import frc.robot.util.Util;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

@Setter
@Getter
public class Elevator extends GenericMotionProfiledSubsystem<ElevatorState> {
  @Getter @Setter private ElevatorState state = ElevatorState.HOME;

  @Getter public final Alert homedAlert = new Alert("NEW HOME SET", Alert.AlertType.kInfo);

  /** Constructor */
  public Elevator(ElevatorIO io, boolean isSim) {
    super(ProfileType.MM_POSITION, ElevatorConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(ElevatorState state) {
    return startEnd(() -> this.state = state, () -> this.state = ElevatorState.HOME);
  }

  private Debouncer homedDebouncer = new Debouncer(.25, DebounceType.kRising);

  public Trigger homedTrigger =
      new Trigger(
          () ->
              homedDebouncer.calculate(
                  (this.state == ElevatorState.HOMING && Math.abs(io.getVelocity()) < .001)));

  public Command zeroSensorCommand() {
    return new InstantCommand(() -> io.zeroSensors());
  }

  public boolean atPosition(double tolerance) {
    return Util.epsilonEquals(io.getPosition(), state.output, Math.max(1, tolerance));
  }

  public Command homedAlertCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> homedAlert.set(true)),
        Commands.waitSeconds(1),
        new InstantCommand(() -> homedAlert.set(false)));
  }

  @AutoLogOutput(key = "Mech2D/FrameComponentPose")
  public Pose3d getPose() {
    return new Pose3d(0.14, 0, 0.13, new Rotation3d());
  }

  @AutoLogOutput(key = "Mech2D/StageComponentPose")
  public Pose3d getStageComponentPose() {
    Transform3d transform = new Transform3d();
    if (io.getPosition() > 0.706) {
      transform = new Transform3d(0, 0, io.getPosition() - 0.706, new Rotation3d());
    }
    return new Pose3d(0.14, 0, 0.169, new Rotation3d()).plus(transform);
  }

  @AutoLogOutput(key = "Mech2D/CarriageComponentPose")
  public Pose3d getCarriageComponentPose() {
    return new Pose3d(0.14, 0, 0.247 + io.getPosition(), new Rotation3d());
  }
}
