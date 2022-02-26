/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.utils.Toggle;

public class RobotDriveCommand extends CommandBase {
  /**
   * Creates a new RobotDriveCommand.
   */
  private final RobotDriveSubsystem driveSubsystem;
  private final Toggle invertDrive, transmissionToggle;

  public RobotDriveCommand(final RobotDriveSubsystem dSubsystem) {
    this.driveSubsystem = dSubsystem;
    this.invertDrive = new Toggle(false);
    this.transmissionToggle = new Toggle(false);

    addRequirements(dSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubsystem.setMotorIdleMode(IdleMode.kCoast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.driveSubsystem.tankDrive(RobotContainer.driverLeft.getY(), RobotContainer.driverRight.getY());

    this.invertDrive.runToggle(RobotContainer.driverRight.getRawButton(1)); // Run toggle
    this.driveSubsystem.setInvertedDrive(this.invertDrive.getState()); // Set inverted based on toggle status
  
    this.transmissionToggle.runToggle(RobotContainer.driverLeft.getRawButton(1));
    
    if(this.transmissionToggle.getState()) {
        this.driveSubsystem.setCurrentGear("high");
    } else {
        this.driveSubsystem.setCurrentGear("low");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    this.driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
