/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;
import frc.robot.utils.Logging;
import frc.robot.utils.EdgeDetector;

public class IndexerSubsystem extends SubsystemBase {
  TalonSRX indexingMotor, intakeMotor;
  DoubleSolenoid intakeExtension;
  DigitalInput indexerFirstPos, indexerSecondPos, fullIndexerSensor;
  TurretSubsystem turretSubsystem;
  Boolean shotInProgress;
  Timer shotInProgressTimer;
  EdgeDetector shooting;
  
  NetworkTableInstance table;
  NetworkTableEntry isFullEntry;
  NetworkTable pidTuningPVs;


  
  
  /**
   * Creates a new IndexerSubsystem.
   */
  
  public IndexerSubsystem(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.indexingMotor = new TalonSRX(Constants.indexerTalonSRX);
    this.intakeMotor = new TalonSRX(Constants.intakeTalonSRX);
    this.intakeExtension = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.intakeOut, Constants.intakeIn);
    indexerFirstPos = new DigitalInput(Constants.indexerFirstPos);
    fullIndexerSensor = new DigitalInput(Constants.fullIndexerSensor);
    this.setupNetworkTables();
    this.shotInProgress = false;
    this.shotInProgressTimer = new Timer();
    this.shotInProgressTimer.reset();
    this.shotInProgressTimer.start();
    this.shooting = new EdgeDetector(true);
  }
  private void setupNetworkTables() {
    this.table = NetworkTableInstance.getDefault();
    this.pidTuningPVs = this.table.getTable("pidTuningPVs");
    this.isFullEntry = this.pidTuningPVs.getEntry("Is Indexer Full");
  }
  private void updateNetworkTables(){
    this.isFullEntry.setBoolean(this.isFull());
  }

  @Override
  public void periodic() {
    String status = String.format(
      "\nFirst sensor status: %s\nFull sensor status: %s",
      ballAtIntake(),
      isFull());
    Logging.debug(status, "indexerStatus");
    this.updateNetworkTables();
    this.updateShotInProgress();
  }

  public void intakeBall() {
    this.intakeBallSpeedOverride(Constants.indexingSpeed, -Constants.intakeSpeed);
  }

  public void intakeBallSpeedOverride(double indexSpeed, double intakeSpeed) {
    if(/*!isFull()*/ true) {
      // intakeRaw(intakeSpeed);
      if(/*ballAtIntake()*/ true) {
        indexerRaw(indexSpeed);
        // this.turretSubsystem.feedShooterRaw(-0.4);
      } else {
        indexerRaw(0);
      }
    } else {
      indexerRaw(0);
      intakeRaw(0);
    }
  }

  private void updateShotInProgress(){
    if (this.shooting.detect(this.isFull())) {
      this.shotInProgressTimer.reset();
    }
    this.shotInProgress = !this.shotInProgressTimer.advanceIfElapsed(Constants.shotInProgressTime);
    this.turretSubsystem.setShotInProgress(this.shotInProgress);
  }

  public void intakeRaw(double speed) {
    this.intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }
  
  public void indexerRaw(double speed) {
    this.indexingMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void lowerIntake() {
    this.intakeExtension.set(DoubleSolenoid.Value.kForward);
  }

  public void raiseIntake() {
    this.intakeExtension.set(DoubleSolenoid.Value.kReverse);
  }

  public void stop() {
    this.intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    this.indexingMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public boolean isShotInProgress(){
    return this.shotInProgress;
  }
  public boolean isFull() {
    return !fullIndexerSensor.get();
  }

  public boolean ballAtIntake() {
    return !indexerFirstPos.get();
  }
}
