package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystem;
import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystem.VoltageState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Intake extends GenericRollerSubsystem<Intake.State> {

  // This is the number of independent roller motors in the subsystem
  // It must equal the size of the voltage arrays in the State enum
  // It also must equal the length of the kMotorIds List in the supplied Constants file
  public static int numRollers = 2;

  @RequiredArgsConstructor
  @Getter
  public enum State implements VoltageState {
    OFF(new double[] {0.0, 0.0}),
    INTAKE(new double[] {10.0, -5.0}),
    EJECT(new double[] {-8.0, 4.0});

    private final double[] output;

    public double getOutput(int index) {
      return this.output[index];
    }
  }

  @Getter @Setter private State state = State.OFF;

  /** Constructor */
  public Intake(IntakeIO io) {
    super("Intake", numRollers, io);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }
}
