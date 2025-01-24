package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PositionTracker;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final PositionTracker positionTracker;

  public Arm(ArmIO io, PositionTracker positionTracker) {
    this.io = io;
    this.positionTracker = positionTracker;

    positionTracker.setArmAngleSupplier(this::getPosition);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier up, DoubleSupplier down) {
    return runEnd(
        () -> io.setVoltage((up.getAsDouble() - down.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  @AutoLogOutput(key = "Mech2D/ArmComponentPose")
  public Pose3d getArmComponentPose() {
    return positionTracker
        .getCarriagePose()
        .plus(new Transform3d(0.083, 0, 0, new Rotation3d()))
        .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getPosition(), 0)));
  }

  @AutoLogOutput(key = "Mech2D/ClawComponentPose")
  public Pose3d getClawComponentPose() {
    return getArmComponentPose().plus(new Transform3d(0.2585, 0, 0, new Rotation3d()));
  }
}
