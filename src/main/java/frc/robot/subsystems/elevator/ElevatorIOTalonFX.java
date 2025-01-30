package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * This Elevator implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
  protected final TalonFX elevator = new TalonFX(ELEVATORCANID);
  protected final StatusSignal<Angle> positionRot = elevator.getPosition();
  protected final StatusSignal<AngularVelocity> velocityRotPerSec = elevator.getVelocity();
  protected final StatusSignal<Voltage> appliedVolts = elevator.getMotorVoltage();
  protected final StatusSignal<Current> currentAmps = elevator.getSupplyCurrent();

  protected final VoltageOut voltageRequest = new VoltageOut(0.0);
  protected final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = CURRENTLIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> elevator.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    elevator.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionMeters = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityMetersPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    elevator.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setHeight(double height) {
    elevator.setControl(motionMagic.withPosition(heightToRotations(height)));
  }

  protected double heightToRotations(double height) {
    return ELEVATORDRUMRADIUS * Math.PI * 2 / height;
  }
}
