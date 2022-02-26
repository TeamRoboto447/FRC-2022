/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.utils.MovingAverage;
import frc.robot.utils.PID;
import frc.robot.utils.ff.FFbase;

public class TurnToAngle extends CommandBase {

  private final MovingAverage averageAngle;

  private final PID drivePID;
  private final RobotDriveSubsystem driveSubsystem;
  private final double targetAngle;

  public TurnToAngle(RobotDriveSubsystem dSubsystem, double targetAngle) {
    this.driveSubsystem = dSubsystem;
    this.targetAngle = targetAngle;
    addRequirements(dSubsystem);

    this.drivePID = new PID.PIDBuilder(this.targetAngle, Constants.drivekP, Constants.drivekI, Constants.drivekD)
      .FF(new FFbase()).Name("drive").build();
    this.averageAngle = new MovingAverage(50);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubsystem.setMotorIdleMode(IdleMode.kBrake);
    this.driveSubsystem.resetEncoders();
    this.driveSubsystem.setCurrentGear("low");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double PV = this.driveSubsystem.getHeading();

    this.averageAngle.push(PV);

    double leftSpeed = this.drivePID.run(PV), rightSpeed = this.drivePID.run(PV);
    this.driveSubsystem.tankDrive(leftSpeed, -rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.targetAngle - 2 < this.averageAngle.getAverage()
        && this.averageAngle.getAverage() < this.targetAngle + 2;
  }
}
