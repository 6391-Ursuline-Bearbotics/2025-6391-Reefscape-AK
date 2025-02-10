package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ArmIOSim extends GenericMotionProfiledSubsystemIOImpl implements ArmIO {
  private MechanismLigament2d armLigament;

  public ArmIOSim(MechanismLigament2d armLigament) {
    super(ArmConstants.kSubSysConstants, true);

    this.armLigament = armLigament;
  }

  @Override
  public void updateInputs(GenericMotionProfiledIOInputs inputs) {
    super.updateInputs(inputs);

    armLigament.setAngle(Units.rotationsToDegrees(super.getPosition()) + 270);
  }
}
