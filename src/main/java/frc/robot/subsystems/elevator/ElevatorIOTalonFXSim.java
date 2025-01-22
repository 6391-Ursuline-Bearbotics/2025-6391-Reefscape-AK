package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class ElevatorIOTalonFXSim extends ElevatorIOTalonFX {
  private TalonFXSimState elevatorSimMotor;
  private final DCMotor elevatorMotor = DCMotor.getFalcon500(1);
  private final MechanismLigament2d ligament;

  private double appliedVolts = 0.0;
  private double appliedPosition = 0.0;

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorMotor,
          ELEVATORGEARING,
          CARRIAGEMASS,
          ELEVATORDRUMRADIUS,
          MINELEVATORHEIGHTMETERS,
          MAXELEVATORHEIGHTMETERS,
          true,
          0,
          0.01,
          0.0);

  public ElevatorIOTalonFXSim(MechanismLigament2d elevatorLigament) {
    super();
    elevatorSimMotor = elevator.getSimState();
    this.ligament = elevatorLigament;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);

    elevatorSimMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    inputs.positionRad = elevatorSim.getPositionMeters();
    inputs.velocityRadPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    ligament.setLength(elevatorSim.getPositionMeters());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPosition(double meters) {
    appliedPosition = MathUtil.clamp(meters, MINELEVATORHEIGHTMETERS, MAXELEVATORHEIGHTMETERS);
  }
}
