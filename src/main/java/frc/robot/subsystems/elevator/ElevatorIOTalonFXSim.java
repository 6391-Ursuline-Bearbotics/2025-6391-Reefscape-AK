package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;

public class ElevatorIOTalonFXSim implements ElevatorIOTalonFX {
  private var elevatorSim = 

  private double appliedVolts = 0.0;
  private double appliedPosition = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPosition(double rotations) {
    appliedVolts = MathUtil.clamp(rotations, -12.0, 12.0);
  }
}
