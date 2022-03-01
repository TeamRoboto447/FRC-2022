/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controlmaps.OperatorMap;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeCommand extends CommandBase {
  IndexerSubsystem indexerSubsystem;
  /**
   * Creates a new IntakeCommand.
   */
  public IntakeCommand(IndexerSubsystem iSubsystem) {
    this.indexerSubsystem = iSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.operator.getRawAxis(OperatorMap.lJoyY) < -0.5) {
      this.indexerSubsystem.lowerIntake();
    } else if(RobotContainer.operator.getRawAxis(OperatorMap.lJoyY) > 0.5) {
      this.indexerSubsystem.raiseIntake();
    }

    if(RobotContainer.operator.getRawButton(OperatorMap.X)) {
      intake();
    } else if(RobotContainer.operator.getRawButton(OperatorMap.RB)) {
      reverseIntake();
    } else {
      stopIntake();
    }

    if(!RobotContainer.operator.getRawButton(OperatorMap.X) && RobotContainer.operator.getRawButton(OperatorMap.LT)) {
      feedShooter();
    } else if(RobotContainer.operator.getRawButton(OperatorMap.LB)) {
      reverseIndexer();
    } else if(!RobotContainer.operator.getRawButton(OperatorMap.X)) {
      stopIndexer();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.indexerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  double indexerSpeed = Constants.indexingSpeed;
  double intakeSpeed = Constants.intakeSpeed;

  /*
  private void runIntake() {
    double speed = -this.intakeSpeed;
    this.indexerSubsystem.intakeRaw(speed);
  }
  */

  private void reverseIntake() {
    double speed = this.intakeSpeed;
    this.indexerSubsystem.intakeRaw(speed);
  }

  private void stopIntake() {
    this.indexerSubsystem.intakeRaw(0);
  }

  // private boolean shooterSpunUp = false;

  private void stopIndexer() {
    // this.shooterSpunUp = false;
    this.indexerSubsystem.indexerRaw(0);
  }

  private void reverseIndexer() {
    // this.shooterSpunUp = false;
    this.indexerSubsystem.indexerRaw(-this.indexerSpeed);
  }

  private void intake() {
    this.indexerSubsystem.intakeBall();
  }


  private void feedShooter() {
    // if(this.turretSubsystem.shooterAtSpeed()) this.shooterSpunUp = true;
    this.indexerSubsystem.indexerRaw(-0.75);
  }
}
