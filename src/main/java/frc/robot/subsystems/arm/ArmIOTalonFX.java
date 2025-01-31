package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
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

/** This Arm implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60. */
public class ArmIOTalonFX implements ArmIO {
  protected final TalonFX arm = new TalonFX(ArmCANID);
  protected final StatusSignal<Angle> positionRot = arm.getPosition();
  protected final StatusSignal<AngularVelocity> velocityRotPerSec = arm.getVelocity();
  protected final StatusSignal<Voltage> appliedVolts = arm.getMotorVoltage();
  protected final StatusSignal<Current> currentAmps = arm.getSupplyCurrent();

  protected final VoltageOut voltageRequest = new VoltageOut(0.0);
  protected final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  public ArmIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = CURRENTLIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> arm.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    arm.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    arm.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setAngle(double angle) {
    arm.setControl(motionMagic.withPosition(angle / 360.0));
  }
}
