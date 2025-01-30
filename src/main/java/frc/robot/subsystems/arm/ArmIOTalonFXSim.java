package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class ArmIOTalonFXSim extends ArmIOTalonFX {
  private TalonFXSimState armSimMotor;
  private final MechanismLigament2d ligament;

  private double appliedVolts = 0.0;
  private double appliedPosition = 0.0;

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          ARMGEARING,
          SingleJointedArmSim.estimateMOI(ARMLENGTH, ARMMASS),
          ARMLENGTH,
          MINARMANGLE,
          MAXARMANGLE,
          true,
          STARTINGANGLE,
          ENCODERPULSEDISTANCE,
          0.0);

  public ArmIOTalonFXSim(MechanismLigament2d armLigament) {
    super();
    armSimMotor = arm.getSimState();
    this.ligament = armLigament;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    super.updateInputs(inputs);

    armSimMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    inputs.positionRad = armSim.getAngleRads();
    inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = armSim.getCurrentDrawAmps();
    ligament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()) + 270);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setAngle(double angle) {
    appliedPosition = MathUtil.clamp(Units.degreesToRadians(angle), MINARMANGLE, MAXARMANGLE);
    armSimMotor.setRawRotorPosition(Units.degreesToRadians(angle) / 360.0);
  }
}
