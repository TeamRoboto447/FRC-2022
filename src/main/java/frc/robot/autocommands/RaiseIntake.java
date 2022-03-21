/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class RaiseIntake extends CommandBase {
  private final IndexerSubsystem indexerSubsystem;
  public RaiseIntake(IndexerSubsystem iSubsystem) {
    this.indexerSubsystem = iSubsystem;
    addRequirements(iSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.indexerSubsystem.raiseIntake();
    
    // if(!this.wasLookingAtBall) {
    //   this.ballCount++;
    //   this.wasLookingAtBall = true;
    //   // System.out.println("Ball In, ball count = " + this.ballCount);
    // } else if(this.wasLookingAtBall) {
    //   // System.out.println("Ball Into Index");
    //   this.wasLookingAtBall = false;
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = true; //this.indexerSubsystem.isFull() || this.ballCount >= this.expectedBalls;
    // if (finished){
    //   System.out.println("IntakeBalls finished");
    // }
    return finished;
  }
}
