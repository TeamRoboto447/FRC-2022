/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndRev extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private double scanAngle;
  private final double scanSpeed;

  public AimAndRev(TurretSubsystem tSubsystem, boolean scanRight, double scanSpeed) {
    this.turretSubsystem = tSubsystem;
    this.scanAngle = scanRight ? -45 : 45;
    this.scanSpeed = scanSpeed;
    addRequirements(tSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.turretSubsystem.enableTargetting(true);
    if (!this.turretSubsystem.validTarget) {
      if (this.turretSubsystem.getTurretPos() < -40) {
        this.scanAngle = 45;
      } else if (this.turretSubsystem.getTurretPos() > 40) {
        this.scanAngle = -45;
      }
      this.turretSubsystem.turnToAngle(this.scanAngle, this.scanSpeed);
      this.spinUp();
    } else {
      this.turretSubsystem.turnToTarget(0.5);
      double currentDistance = this.turretSubsystem.getDistance();
      double shooterSpeed = this.turretSubsystem.getSpeedFromDist(currentDistance);
      this.turretSubsystem.runShooterAtSpeed(shooterSpeed);
    }
  }
  

  private void spinUp() {
    this.turretSubsystem.runShooterAtSpeed(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.enableTargetting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
