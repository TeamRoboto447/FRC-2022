/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final DoubleSolenoid climberTilt;
  private final CANSparkMax climbMotor;
  private final RobotDriveSubsystem driveSubsystem;

  public ClimberSubsystem(RobotDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.climberTilt = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.climberOut, Constants.climberIn);
    this.climbMotor = new CANSparkMax(Constants.climberSparkMax, MotorType.kBrushless);
    this.climbMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(double speed) {
    this.climbMotor.set(speed);
  }

  public void tiltClimber() {
    this.climberTilt.set(DoubleSolenoid.Value.kForward);
  }

  public void correctClimber() {
    this.climberTilt.set(DoubleSolenoid.Value.kReverse);
  }

  public void stop() {
    this.climbMotor.set(0);
  } 
}
