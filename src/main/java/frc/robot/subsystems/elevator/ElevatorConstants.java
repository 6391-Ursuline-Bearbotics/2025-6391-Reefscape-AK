package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int ELEVATORCANID = 5;
  public static final int CURRENTLIMIT = 40;

  public static final double ELEVATORGEARING = 24.0;
  public static final double CARRIAGEMASS = Units.lbsToKilograms(12.0);
  public static final double ELEVATORDRUMRADIUS = Units.inchesToMeters(1.0);
  public static final double MINELEVATORHEIGHTMETERS = Units.inchesToMeters(12.0);
  public static final double MAXELEVATORHEIGHTMETERS = Units.inchesToMeters(72.0);
}
