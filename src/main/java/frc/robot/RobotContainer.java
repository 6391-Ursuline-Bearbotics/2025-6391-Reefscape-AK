// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmIOTalonFX;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import frc.robot.util.TargetingSystem;
import frc.robot.util.TargetingSystem.ElevatorState;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  TargetingSystem target = new TargetingSystem();

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Arm arm;

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController drv = new CommandXboxController(0);
  private final CommandXboxController op = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Double> speedChooser =
      new LoggedDashboardChooser<>("Swerve Speed");
  ;
  // private final LoggedDashboardChooser<ScoreLevel> reefHeight;
  // private final LoggedDashboardChooser<ReefFace> reefFace;
  // private final LoggedDashboardChooser<ReefPipe> reefPipe;

  private Mechanism2d mechanisms = new Mechanism2d(5, 3);
  private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);

  @SuppressWarnings("unused")
  private MechanismLigament2d fromRobot =
      root.append(
          new MechanismLigament2d(
              "fromRobot", Units.inchesToMeters(5.5), 180, 0, new Color8Bit(Color.kWhite)));

  @SuppressWarnings("unused")
  private MechanismLigament2d elevatorBase =
      root.append(
          new MechanismLigament2d(
              "elevatorBase", Units.inchesToMeters(36), 90, 2, new Color8Bit(Color.kWhite)));

  private MechanismLigament2d elevatorLigament =
      root.append(
          new MechanismLigament2d(
              "elevatorStage", Units.inchesToMeters(10), 90, 4, new Color8Bit(Color.kOrange)));
  private MechanismLigament2d armLigament =
      elevatorLigament.append(
          new MechanismLigament2d(
              "armLigament", Units.inchesToMeters(10), 270, 5, new Color8Bit(Color.kRed)));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        this.vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation),
                new VisionIOQuestNav(VisionConstants.questName));
        elevator = new Elevator(new ElevatorIOTalonFX(), false);
        arm = new Arm(new ArmIOTalonFX(), false, () -> elevator.getCarriageComponentPose());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
        elevator = new Elevator(new ElevatorIOSim(elevatorLigament), true);
        arm = new Arm(new ArmIOSim(armLigament), true, () -> elevator.getCarriageComponentPose());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {}, true);
        arm = new Arm(new ArmIO() {}, true, () -> elevator.getCarriageComponentPose());
        break;
    }

    // QuestNav initialization
    // vision.

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("Right 3 L4"));

    for (String auto : AutoBuilder.getAllAutoNames()) {
      autoChooser.addOption(auto.replace("Right", "Left"), new PathPlannerAuto(auto, true));
    }

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    speedChooser.addDefaultOption("100%", 1.0);
    speedChooser.addOption("95%", 0.95);
    speedChooser.addOption("90%", 0.9);
    speedChooser.addOption("85%", 0.85);
    speedChooser.addOption("80%", 0.8);
    speedChooser.addOption("75%", 0.75);
    speedChooser.addOption("70%", 0.7);
    speedChooser.addOption("65%", 0.65);
    speedChooser.addOption("60%", 0.6);
    speedChooser.addOption("55%", 0.55);
    speedChooser.addOption("50%", 0.5);
    speedChooser.addOption("35%", 0.35);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -drv.getLeftY(), () -> -drv.getLeftX(), () -> -drv.getRightX()));

    // Reset gyro to 0° when start button is pressed
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    drv.start().onTrue(runOnce(resetGyro, drive).ignoringDisable(true));

    // Turtle Mode toggle
    drv.leftBumper().onTrue(drive.toggleTurtleMode());

    // While Right Bumper is held drive robot relative
    drv.rightBumper().whileTrue(DriveCommands.joystickRobotDrive(drive, () -> -drv.getLeftY(), () -> -drv.getLeftX(), () -> -drv.getRightX()));

    op.y().onTrue(target.setTargetLevel(ElevatorState.LEVEL_4));
    op.x().onTrue(target.setTargetLevel(ElevatorState.LEVEL_3));
    op.b().onTrue(target.setTargetLevel(ElevatorState.LEVEL_2));
    op.a().onTrue(target.setTargetLevel(ElevatorState.LEVEL_1));

    op.leftBumper().onTrue(target.moveTargetBranchLeft());
    op.rightBumper().onTrue(target.moveTargetBranchRight());

    op.start()
        .onTrue(
            parallel(
                elevator.setStateCommand(target.getElevatorState()),
                waitUntil(() -> elevator.atPosition(0.1))
                    .andThen(arm.setStateCommand(target.getArmState()))));

    Trigger speedPick =
        new Trigger(
            () -> {
              Double chosenSpeed = speedChooser.get();
              // If chosenSpeed is null, we consider the condition to be false
              if (chosenSpeed == null) {
                return false;
              }
              return drive.maxSpeedPercentage != chosenSpeed;
            });
    speedPick.onTrue(runOnce(() -> drive.setMaxSpeed(speedChooser.get())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
