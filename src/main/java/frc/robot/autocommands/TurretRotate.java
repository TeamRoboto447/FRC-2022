/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretRotate extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final double targetAngle, margin;

  public TurretRotate(TurretSubsystem tSubsystem, double angle, double margin) {
    this.turretSubsystem = tSubsystem;
    this.targetAngle = angle;
    this.margin = margin;
    addRequirements(tSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.turretSubsystem.turnToAngle(this.targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.enableTargetting(false);
    this.turretSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.turretSubsystem.getTurretPos() > this.targetAngle - this.margin) && (this.turretSubsystem.getTurretPos() < this.targetAngle + this.margin);
  }
}
