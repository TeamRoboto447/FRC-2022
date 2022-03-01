/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.controlmaps.OperatorMap;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  // private final RobotDriveSubsystem driveSubsystem;

  public TurretCommand(TurretSubsystem tSubsystem, RobotDriveSubsystem dSubsystem) {
    this.turretSubsystem = tSubsystem;
    // this.driveSubsystem = dSubsystem;
    addRequirements(this.turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.operator.getRawButton(OperatorMap.A)) {
      if (this.turretSubsystem.validTarget) {
        target();
      } else {
        turnManual();
      }
    } else {
      turretLock();
    }

    if (RobotContainer.operator.getRawButton(OperatorMap.RT)) {
      // this.turretSubsystem.enableShooterLogging(true);
      shootAtDistance();
      runFeeder();
    } else if (RobotContainer.operator.getRawButton(OperatorMap.Y)) {
      // this.turretSubsystem.enableShooterLogging(true);
      shootAtSpeed();
      runFeeder();
    } else if (RobotContainer.operator.getRawButton(OperatorMap.LB)) {
      // this.turretSubsystem.enableShooterLogging(false);
      this.turretSubsystem.runShooterRaw(0);
      this.reverseFeeder();
    } else {
      // this.turretSubsystem.enableShooterLogging(false);
      this.turretSubsystem.runShooterRaw(0);
      this.turretSubsystem.stopFeeder();
    }

    /*if (RobotContainer.operator.getRawButton(OperatorMap.LT)) {
      lockDist();
      lockTurret();
    } else {
      unlockDist();
      unlockTurret();
    }*/

  }

  // private void lockDist() {
  //   this.turretSubsystem.lockDistance();
  // }

  // private void unlockDist() {
  //   this.turretSubsystem.unlockDistance();
  // }

  // private void lockTurret() {
  //   this.turretSubsystem.lockTurret();
  // }

  // private void unlockTurret() {
  //   this.turretSubsystem.unlockTurret();
  // }

  private void shootAtDistance() {
    double dist = this.turretSubsystem.getDistance();
    double speed = this.turretSubsystem.getSpeedFromDist(dist);
    speed = this.turretSubsystem.getManualSpeed();
    this.turretSubsystem.runShooterAtSpeed(speed);
  }

  private void shootAtSpeed() {
    double speed = this.turretSubsystem.getManualSpeed();
    this.turretSubsystem.runShooterAtSpeed(speed);
  }

  private void runFeeder() {
    if (/*this.turretSubsystem.shooterAtSpeed() &&*/ RobotContainer.operator.getRawButton(OperatorMap.LT)) {
      this.turretSubsystem.feedShooter();
    } else {
      this.turretSubsystem.stopFeeder();
    }
  }

  private void reverseFeeder() {
    this.turretSubsystem.feedShooterRaw(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void turretLock() {
    this.turretSubsystem.enableTargetting(false);
    int lockPosition = 0;
    this.turretSubsystem.turnToAngle(lockPosition);
  }

  private void turnManual() {
    this.turretSubsystem.enableTargetting(true);
    double position = -Utilities.adjustForDeadzone(RobotContainer.operator.getRawAxis(OperatorMap.lJoyX), 0.025);
    double angle = 90 * position;
    this.turretSubsystem.turnToAngle(angle, 0.5);
  }

  private void target() {
    this.turretSubsystem.enableTargetting(true);
    this.turretSubsystem.turnToTarget();
  }

  // private double calculateAngleOffset() {
  //   double[] speeds = this.driveSubsystem.getFieldRelativeSpeed();
  //   double dist = this.turretSubsystem.getDistance();
  //   double shooterSpeed = this.turretSubsystem.getShooterSpeedFPS();
  //   double targetAngle = this.turretSubsystem.getTargetAngle();
  //   double distm = Units.feetToMeters(dist);
  //   double offsetDistance = dist / shooterSpeed * speeds[1];
  //   double adjustedDist = Math
  //       .sqrt(Math.pow(offsetDistance, 2) + Math.pow(distm, 2) - 2 * offsetDistance * distm * Math.cos(targetAngle));
  //   double offsetAngle = Math.asin(Math.sin(targetAngle) / adjustedDist * distm);
  //   return Math.toDegrees(offsetAngle);
  // }

  // private double calculateSpeedOffset() {
  //   double[] speeds = this.driveSubsystem.getFieldRelativeSpeed();
  //   double dist = this.turretSubsystem.getDistance();
  //   double shooterSpeed = this.turretSubsystem.getShooterSpeedFPS();
  //   return 0;
  // }
}
