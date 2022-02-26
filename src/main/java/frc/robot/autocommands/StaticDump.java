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
import frc.robot.subsystems.TurretSubsystem;

public class StaticDump extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final Timer countDown, overallTimer, distanceTimer;
  private final double maxRunTime;
  private final int ballsToShoot;
  private final double angle;
  private final double startingDist;
  private int ballsShot = 0;
  private boolean wasLookingAtBall = false;

  public StaticDump(TurretSubsystem tSubsystem, IndexerSubsystem iSubsystem, double startingDist, double angle, double maxTime,
      int ballsToShoot) {
    this.turretSubsystem = tSubsystem;
    this.indexerSubsystem = iSubsystem;
    this.angle = angle;
    this.startingDist = startingDist;

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
  }


  boolean onTarget = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.turretSubsystem.enableTargetting(true);
    if (!this.onTarget) {
      this.turretSubsystem.turnToAngle(this.angle);
      if(this.angle-2 < this.turretSubsystem.getTurretPos() && this.turretSubsystem.getTurretPos() < this.angle+2) {

      }
      spinUp();
    } else {
      this.turretSubsystem.turnToTarget(0.5);
      if (this.turretSubsystem.onTarget()) {
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

  private boolean spunUp = false;

  private void shoot() {
    double currentDistance = this.startingDist;
    double shooterSpeed = this.turretSubsystem.getSpeedFromDist(currentDistance);
    this.turretSubsystem.runShooterAtSpeed(shooterSpeed);
    this.indexerSubsystem.intakeRaw(-Constants.intakeSpeed);
    if (this.turretSubsystem.shooterAtSpeed() && !this.spunUp) {
      this.spunUp = true;
      this.distanceTimer.start();
    }

    if (this.spunUp) {
      this.turretSubsystem.lockDistance();
      if (this.distanceTimer.get() > 0.3) {
        this.turretSubsystem.feedShooter();
        this.indexerSubsystem.indexerRaw(0.35);
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
