package frc.robot.subsystems.Intake;

import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOImpl;

public class IntakeIOSim extends GenericRollerSubsystemIOImpl implements IntakeIO {

  public IntakeIOSim() {
    super(IntakeConstants.kSubSysConstants, true);
  }
}
