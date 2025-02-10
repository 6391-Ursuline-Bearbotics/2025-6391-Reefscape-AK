package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Ports;
import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemConstants;
import java.util.List;

/** Add your docs here. */
public final class IntakeConstants {

  public static final GenericRollerSubsystemConstants kSubSysConstants =
      new GenericRollerSubsystemConstants();

  static {
    kSubSysConstants.kName = "Intake";

    kSubSysConstants.kMotorIDs = List.of(Ports.INTAKE);

    kSubSysConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kSubSysConstants.kMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kSubSysConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    kSubSysConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

    kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
    kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor simulation
    kSubSysConstants.simMotorModelSupplier = () -> DCMotor.getFalcon500(1);
    kSubSysConstants.simReduction = (18.0 / 12.0);
    kSubSysConstants.simMOI = 0.001;
  }
}
