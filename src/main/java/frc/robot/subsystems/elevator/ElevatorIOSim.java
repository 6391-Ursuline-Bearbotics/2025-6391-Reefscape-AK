package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ElevatorIOSim extends GenericMotionProfiledSubsystemIOImpl implements ElevatorIO {
  private MechanismLigament2d elevatorLigament;

  public ElevatorIOSim(MechanismLigament2d elevatorLigament) {
    super(ElevatorConstants.kSubSysConstants, true);

    this.elevatorLigament = elevatorLigament;
  }

  @Override
  public void updateInputs(GenericMotionProfiledIOInputs inputs) {
    super.updateInputs(inputs);

    elevatorLigament.setLength(super.getPosition());
  }
}
