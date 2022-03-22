/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controlmaps.OperatorMap;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.RobotDriveSubsystem;

public class ClimbCommand extends CommandBase {

  private final ClimberSubsystem climberSubsystem;
  private final RobotDriveSubsystem driveSubsystem;
  private final Timer stepDelayTimer;
  private int currentClimbStep = 0;

  public ClimbCommand(ClimberSubsystem cSubsystem, RobotDriveSubsystem dSubsystem) {
    this.climberSubsystem = cSubsystem;
    this.driveSubsystem = dSubsystem;
    this.stepDelayTimer = new Timer();
    addRequirements(this.climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private boolean withinTiltMargin() {
    return this.driveSubsystem.getTilt() < Constants.autoClimbTiltMargin
        && this.driveSubsystem.getTilt() > -Constants.autoClimbTiltMargin;
  }

  private boolean stepEnded = false;

  private void nextStep() {
    if (!stepEnded)
      this.stepDelayTimer.reset();
    this.stepDelayTimer.start();
    this.stepEnded = true;
    if (this.stepDelayTimer.get() > Constants.autoClimStepDelay) {
      this.currentClimbStep++;
      this.stepEnded = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.operator.getRawButton(OperatorMap.rJoyPress)) {
      // System.out.print("Current Climb Step: ");
      System.out.println(currentClimbStep);
      switch (currentClimbStep) {
        case 0:
          this.climberSubsystem.tiltClimber();
          this.nextStep();
          break;
        case 1:
          if (this.climberSubsystem.getWind() >= 210)
            this.nextStep();
          else
            this.climberSubsystem.climb(1);
          break;
        case 2:
          if (withinTiltMargin())
            this.nextStep();
          else
            this.climberSubsystem.correctClimber();
          break;
        case 3:
          if (this.climberSubsystem.getWind() <= 0)
            this.nextStep();
          else if(this.driveSubsystem.getTilt() < 0)
            this.climberSubsystem.climb(-1);
          break;
        case 4:
          if (this.climberSubsystem.getWind() >= 130)
            this.currentClimbStep++;
          else
            this.climberSubsystem.climb(1);
          break;
        case 5:
          if (this.climberSubsystem.getWind() <= 50)
            this.currentClimbStep++;
          else if(this.driveSubsystem.getTilt() < 0)
            this.climberSubsystem.climb(-1);
          break;
        case 6:
          this.currentClimbStep = 0;
          break;
        default:
          // System.out.println("Climber Steps Done");
          break;

      }

    } else if (RobotContainer.driverLeft.getRawButton(10)) {
      this.currentClimbStep = 0;
    }

    if (RobotContainer.operator.getRawButton(OperatorMap.start) && this.climberSubsystem.allowedToClimb(1)) {
      this.climberSubsystem.climb(1);
    } else if (RobotContainer.operator.getRawButton(OperatorMap.back) && this.climberSubsystem.allowedToClimb(-1)) { // &&
                                                                                                                     // this.driveSubsystem.getTilt()
                                                                                                                     // <
                                                                                                                     // 1.2
      this.climberSubsystem.climb(-1);
    } else if (!RobotContainer.operator.getRawButton(OperatorMap.rJoyPress)) {
      this.climberSubsystem.stop();
    }

    if (RobotContainer.operator.getPOV() == 90) {
      this.climberSubsystem.correctClimber();
    } else if (RobotContainer.operator.getPOV() == 270) {
      this.climberSubsystem.tiltClimber();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
