/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndShoot extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final RobotDriveSubsystem driveSubsystem;
  private final Timer countDown, overallTimer, distanceTimer;
  private final double maxRunTime;
  private final int ballsToShoot;
  private double scanAngle;
  private final double scanSpeed;
  private int ballsShot = 0;
  private boolean wasLookingAtBall = false;
  private boolean shooting = false;

  public AimAndShoot(TurretSubsystem tSubsystem, IndexerSubsystem iSubsystem, RobotDriveSubsystem dSubsystem, boolean scanRight, double scanSpeed,
      double maxTime, int ballsToShoot) {
    this.turretSubsystem = tSubsystem;
    this.indexerSubsystem = iSubsystem;
    this.driveSubsystem = dSubsystem;
    this.scanAngle = scanRight ? -45 : 45;
    this.scanSpeed = scanSpeed;

    this.distanceTimer = new Timer();

    this.countDown = new Timer();

    this.overallTimer = new Timer();

    this.maxRunTime = maxTime;
    this.ballsToShoot = ballsToShoot;

    addRequirements(tSubsystem);
    addRequirements(iSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.distanceTimer.stop();
    this.distanceTimer.reset();
    this.countDown.stop();
    this.countDown.reset();
    this.overallTimer.reset();
    this.overallTimer.start();
    this.indexerSubsystem.raiseIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.driveSubsystem.stop();
    this.turretSubsystem.enableTargetting(true);
    if (!this.turretSubsystem.validTarget) {
      if (this.turretSubsystem.getTurretPos() < -40) {
        this.scanAngle = 45;
      } else if (this.turretSubsystem.getTurretPos() > 40) {
        this.scanAngle = -45;
      }
      this.turretSubsystem.turnToAngle(this.scanAngle, this.scanSpeed);
      spinUp();
    } else {
      this.turretSubsystem.turnToTarget(0.5);
      if (this.turretSubsystem.onTarget() || this.shooting) {
        this.shooting = true;
        shoot();
      } else {
        spinUp();
      }
    }

    if (!this.wasLookingAtBall && this.indexerSubsystem.isFull()) {
      this.ballsShot++;
      this.wasLookingAtBall = true;
    } else if (this.wasLookingAtBall && !this.indexerSubsystem.isFull()) {
      this.wasLookingAtBall = false;
    }

    if (this.ballsShot >= this.ballsToShoot) {
      this.countDown.reset();
      this.countDown.start();
    }
  }

  private void shoot() {
    double currentDistance = this.turretSubsystem.getDistance();
    double shooterSpeed = this.turretSubsystem.getSpeedFromDist(currentDistance);
    this.turretSubsystem.runShooterAtSpeed(shooterSpeed);
    // this.indexerSubsystem.intakeRaw(-Constants.intakeSpeed);
    this.distanceTimer.start();
    if (true) { //this.turretSubsystem.shooterAtSpeed()
      if (this.distanceTimer.get() > 1.5) {
        this.turretSubsystem.lockDistance();
        this.turretSubsystem.feedShooter();
        this.indexerSubsystem.indexerRaw(0.75);
      }
    }

  }

  private void spinUp() {
    this.turretSubsystem.runShooterAtSpeed(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.enableTargetting(false);
    this.turretSubsystem.stop();
    this.indexerSubsystem.stop();
    this.turretSubsystem.unlockDistance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // this.countDown.get() >= 0.5 ||
    return this.overallTimer.get() >= this.maxRunTime;
  }
}
