package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
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

  @AutoLogOutput(key = "Mech2D")
  public Pose3d getPose() {
    return new Pose3d(0.14, 0, 0.13, new Rotation3d());
  }

  @AutoLogOutput(key = "Mech2D")
  public Pose3d getStageComponentPose() {
      Transform3d transform = new Transform3d();
      if (getPosition() > 0.706) {
          transform = new Transform3d(0, 0, getPosition() - 0.706, new Rotation3d());
      }
      return new Pose3d(0.14, 0, 0.169, new Rotation3d()).plus(transform);
  }

  @AutoLogOutput(key = "Mech2D")
  public Pose3d getCarriageComponentPose() {
      return new Pose3d(0.14, 0, 0.247 + getPosition(), new Rotation3d());
  }
}
