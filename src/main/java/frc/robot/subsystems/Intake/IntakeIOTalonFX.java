package frc.robot.subsystems.Intake;

import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOImpl;

public class IntakeIOTalonFX extends GenericRollerSubsystemIOImpl implements IntakeIO {

  public IntakeIOTalonFX() {
    super(IntakeConstants.kSubSysConstants, false);
  }
}
