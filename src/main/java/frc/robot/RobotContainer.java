/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
// import frc.robot.autocommands.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
  // public final TurretCommand turretCommand = new TurretCommand(turretSubsystem, driveSubsystem);
  public final IntakeCommand intakeCommand = new IntakeCommand(indexerSubsystem);
  public final ClimbCommand climbCommand = new ClimbCommand(climberSubsystem);

  public final SendableChooser<Command> autonomousSelector;

  public static Joystick driverRight = new Joystick(0);
  public static Joystick driverLeft = new Joystick(1);
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

  // private final boolean scanLeft = false;
  // private final boolean scanRight = true;

  // SequentialCommandGroup threeBallAuto = new SequentialCommandGroup(
  //     new AimAndDump(this.turretSubsystem, this.indexerSubsystem, scanLeft, 0.5, 6, 3),
  //     new DriveToPosition(this.driveSubsystem, "high", Utilities.feetToEncoder(2.5), 0.2, 1)
  //   );

  // SequentialCommandGroup threeBallAutoManualAim = new SequentialCommandGroup(
  //     new StaticDump(this.turretSubsystem, this.indexerSubsystem, 10.11, 0, 4, 3),
  //     new DriveToPosition(this.driveSubsystem, "high", Utilities.feetToEncoder(2.5), 0.2, 1)
  //     );

  // SequentialCommandGroup threeBallAutoPush = new SequentialCommandGroup(
  //     new AimAndDump(this.turretSubsystem, this.indexerSubsystem, scanLeft, 0.5, 6, 3),
  //     new DriveToPosition(this.driveSubsystem, "low", Utilities.feetToEncoder(-2), 0.4, 1),
  //     new DriveToPosition(this.driveSubsystem, "high", Utilities.feetToEncoder(5), 0.2, 1)
  //     );

  // SequentialCommandGroup sixBallAuto = new SequentialCommandGroup(
  //     new AimAndDump(this.turretSubsystem, this.indexerSubsystem, scanLeft, 0.4, 4, 3),
  //     new ParallelRaceGroup(
  //       new KeepShooterRevved(this.turretSubsystem),
  //       new IntakeBalls(this.indexerSubsystem, 3),
  //       new DriveToPosition(this.driveSubsystem, "high", Utilities.feetToEncoder(16), 0.6, 0.8)
  //     ),
  //     new AimAndDump(this.turretSubsystem, this.indexerSubsystem, scanLeft, 0.3, 10, 3));

  // SequentialCommandGroup fiveBallAuto = new SequentialCommandGroup(
  //     new AimAndDump(this.turretSubsystem, this.indexerSubsystem, scanLeft, 0.4, 4, 3),
  //     new ParallelRaceGroup(new KeepShooterRevved(this.turretSubsystem), new IntakeBalls(this.indexerSubsystem, 2),
  //         new DriveToPosition(this.driveSubsystem, "high", Utilities.feetToEncoder(4), 0.6, 1)),
  //     // new ParallelRaceGroup(new KeepShooterRevved(this.turretSubsystem), new IntakeBalls(this.indexerSubsystem, 2),
  //     //     new DriveToPosition(this.driveSubsystem, "high", Utilities.feetToEncoder(-3), 0.6, 1)),
  //     new AimAndDump(this.turretSubsystem, this.indexerSubsystem, scanLeft, 0.3, 10, 2));

  private void addAutonomousCommands() {
    // this.autonomousSelector.setDefaultOption("3 Ball Auto", this.threeBallAuto);
    // // this.autonomousSelector.addOption("Three Ball Auto (Manual Target)",
    // // this.threeBallAutoManualAim);
    // this.autonomousSelector.addOption("3 Ball Auto With Assist", this.threeBallAutoPush);
    // this.autonomousSelector.addOption("3+ Ball Auto", this.sixBallAuto);
    // this.autonomousSelector.addOption("5 Ball Auto", this.fiveBallAuto);
    Shuffleboard.getTab("Autonomous").add(this.autonomousSelector);
  }

  private void setDefaultCommands() {

    this.driveSubsystem.setDefaultCommand(this.driveCommand);
    // this.turretSubsystem.setDefaultCommand(this.turretCommand);
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
