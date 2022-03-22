/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
  private final DoubleSolenoid climberTilt;
  private final CANSparkMax climbMotor;
  private final RobotDriveSubsystem driveSubsystem;
  private final NetworkTableEntry overrideClimb;

  DoubleSupplier tiltSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return driveSubsystem.getTilt();
    }
  };

  DoubleSupplier windSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return getWind();
    }
  };

  BooleanSupplier allowedToClimbSupplier = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return allowedToClimb(0);
    }
  };

  public ClimberSubsystem(RobotDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.climberTilt = new DoubleSolenoid(Constants.pneumaticsType, Constants.climberOut, Constants.climberIn);
    this.climbMotor = new CANSparkMax(Constants.climberSparkMax, MotorType.kBrushless);
    this.climbMotor.setIdleMode(IdleMode.kBrake);
    this.climbMotor.getEncoder().setPosition(0);
    this.overrideClimb = Shuffleboard.getTab("Climb Debug").add("Override Climb Limiter", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    this.overrideClimb.setBoolean(false);
    Shuffleboard.getTab("Climb Debug").addNumber("Current Tilt (Front to Back)", tiltSupplier);
    Shuffleboard.getTab("Climb Debug").addNumber("Current Wind", windSupplier);
    Shuffleboard.getTab("Climb Debug").addBoolean("Allowed to Climb", allowedToClimbSupplier);
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

  public double getWind() {
    return this.climbMotor.getEncoder().getPosition();
  }

  public boolean allowedToClimb(double direction) {
    boolean allowed = false;
    if (direction < 0) {
      // allowed = true;
      allowed = getWind() > Constants.lowerWindLimit;
    } else if (direction > 0) {
      // allowed = true;
      allowed = getWind() < Constants.upperWindLimit;
    } else {
      allowed = Constants.lowerWindLimit < getWind() && getWind() < Constants.upperWindLimit;
    }
    
    return allowed || (this.overrideClimb.getBoolean(false) || RobotContainer.driverLeft.getRawButton(9) || true);
  }

  public void stop() {
    this.climbMotor.set(0);
  }
}
