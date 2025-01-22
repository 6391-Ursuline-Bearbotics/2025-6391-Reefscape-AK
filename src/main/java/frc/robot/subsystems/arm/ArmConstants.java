package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final int ArmCANID = 6;
  public static final int CURRENTLIMIT = 40;

  public static final double ARMGEARING = 24.0;
  public static final double ARMMASS = Units.lbsToKilograms(12.0);
  public static final double ARMLENGTH = Units.inchesToMeters(12.0);
  public static final double MINARMANGLE = Units.degreesToRadians(-40.0);
  public static final double MAXARMANGLE = Units.degreesToRadians(90.0);
  public static final double STARTINGANGLE = Units.degreesToRadians(90.0);
  public static final double ENCODERPULSEDISTANCE = 2.0 * Math.PI / 4096;
}
