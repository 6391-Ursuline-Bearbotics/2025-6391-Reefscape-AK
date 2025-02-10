package frc.robot;

import frc.robot.util.drivers.CanDeviceId;

public class Ports {
  /*
   * LIST OF CHANNEL AND CAN IDS
   */

  /* SUBSYSTEM CAN DEVICE IDS */
  public static final CanDeviceId INTAKE = new CanDeviceId(4, "rio");

  public static final CanDeviceId CLIMBER = new CanDeviceId(5, "rio");

  public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(1, "rio");
  public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(2, "rio");

  public static final CanDeviceId ARM_MAIN = new CanDeviceId(3, "rio");
  public static final CanDeviceId ARM_CANCODER = new CanDeviceId(9, "rio");
}
