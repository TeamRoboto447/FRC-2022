/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.autocommands.AimAndRev;
import frc.robot.autocommands.AimAndShoot;
import frc.robot.autocommands.TurretRotate;
import frc.robot.autocommands.DriveToPosition;
import frc.robot.autocommands.IntakeBalls;
import frc.robot.autocommands.InvertDrive;
import frc.robot.autocommands.LowerDump;
import frc.robot.autocommands.RaiseIntake;
import frc.robot.commands.*;
// import frc.robot.autocommands.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final BlingSubsystem blingSubsystem = new BlingSubsystem();

  public final RobotDriveSubsystem driveSubsystem = new RobotDriveSubsystem();
  public final TurretSubsystem turretSubsystem = new TurretSubsystem(driveSubsystem);
  public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem(turretSubsystem);
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem(driveSubsystem);

  public final RobotDriveCommand driveCommand = new RobotDriveCommand(driveSubsystem);
  public final TurretCommand turretCommand = new TurretCommand(turretSubsystem, driveSubsystem);
  public final IntakeCommand intakeCommand = new IntakeCommand(indexerSubsystem);
  public final ClimbCommand climbCommand = new ClimbCommand(climberSubsystem, driveSubsystem);

  public final SendableChooser<Command> autonomousSelector;

  public static Joystick driverLeft = new Joystick(0);
  public static Joystick driverRight = new Joystick(1);
  public static Joystick operator = new Joystick(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Set default commands
    setDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Set up autonomous selector
    this.autonomousSelector = new SendableChooser<>();
    addAutonomousCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  private final boolean scanLeft = false;
  // private final boolean scanRight = true;

  SequentialCommandGroup singleBall = new SequentialCommandGroup(
      new RaiseIntake(this.indexerSubsystem),
      new DriveToPosition(this.driveSubsystem, "low", Utilities.inchToEncoder(65), 0.65, 1),
      new AimAndShoot(this.turretSubsystem, this.indexerSubsystem, this.driveSubsystem, scanLeft, 0.5, 5, 1));

  SequentialCommandGroup twoUpper = new SequentialCommandGroup(
      new RaiseIntake(this.indexerSubsystem),
      new DriveToPosition(this.driveSubsystem, "low", Utilities.inchToEncoder(65), 0.65, 1),
      new AimAndShoot(this.turretSubsystem, this.indexerSubsystem, this.driveSubsystem, scanLeft, 0.5, 5, 1),
      new ParallelRaceGroup(
          new DriveToPosition(this.driveSubsystem, "low", Utilities.inchToEncoder(30), 0.5, 1),
          new IntakeBalls(this.indexerSubsystem, 1)),
      new AimAndShoot(this.turretSubsystem, this.indexerSubsystem, this.driveSubsystem, scanLeft, 0.5, 5, 1));

  SequentialCommandGroup twoLower = new SequentialCommandGroup(
      new ParallelRaceGroup(
          new DriveToPosition(this.driveSubsystem, "low", Utilities.inchToEncoder(95), 0.65, 1),
          new IntakeBalls(this.indexerSubsystem, 1)),
      new InvertDrive(this.driveSubsystem),
      new DriveToPosition(this.driveSubsystem, "low", Utilities.inchToEncoder(95), 0.65, 1),
      new TurretRotate(this.turretSubsystem, 0, 5),
      new LowerDump(this.turretSubsystem, this.indexerSubsystem));

  SequentialCommandGroup twoUpperExperiment = new SequentialCommandGroup(
      new RaiseIntake(this.indexerSubsystem),
      new DriveToPosition(this.driveSubsystem, "low", Utilities.inchToEncoder(65), 0.65, 1),
      new AimAndShoot(this.turretSubsystem, this.indexerSubsystem, this.driveSubsystem, scanLeft, 0.5, 5, 1),
      new ParallelRaceGroup(
          new DriveToPosition(this.driveSubsystem, "low", Utilities.inchToEncoder(30), 0.5, 1),
          new IntakeBalls(this.indexerSubsystem, 1),
          new AimAndRev(this.turretSubsystem, scanLeft, 1)),
      new AimAndShoot(this.turretSubsystem, this.indexerSubsystem, this.driveSubsystem, scanLeft, 0.5, 5, 1));

  private void addAutonomousCommands() {
    this.autonomousSelector.addOption("1 Ball Upper", this.singleBall);
    this.autonomousSelector.setDefaultOption("2 Ball Upper", this.twoUpper);
    this.autonomousSelector.addOption("2 Ball Lower", this.twoLower);
    // this.autonomousSelector.addOption("Three Ball Auto (Manual Target)",
    // this.threeBallAutoManualAim);
    // this.autonomousSelector.addOption("3 Ball Auto With Assist",
    // this.threeBallAutoPush);
    // this.autonomousSelector.addOption("3+ Ball Auto", this.sixBallAuto);
    // this.autonomousSelector.addOption("5 Ball Auto", this.fiveBallAuto);
    Shuffleboard.getTab("Autonomous").add(this.autonomousSelector);
  }

  private void setDefaultCommands() {

    this.driveSubsystem.setDefaultCommand(this.driveCommand);
    this.turretSubsystem.setDefaultCommand(this.turretCommand);
    this.indexerSubsystem.setDefaultCommand(this.intakeCommand);
    this.climberSubsystem.setDefaultCommand(this.climbCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return this.autonomousSelector.getSelected();
  }
}
