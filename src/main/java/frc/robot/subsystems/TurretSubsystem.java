/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Logging;
import frc.robot.utils.MovingAverage;
import frc.robot.utils.PID;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.utils.ff.LinearFF;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

// import java.io.File;
// import java.io.FileWriter;
// import java.io.IOException;
// import java.nio.file.Path;

// import com.opencsv.CSVWriter;

//import java.lang.Math;

public class TurretSubsystem extends SubsystemBase {
  // CSVWriter shooterLogging;
  boolean shooterWriterActive, enableShooterLogging = false;

  MovingAverage distanceAverage, shooterSpeedAverage;

  NetworkTableInstance table;
  NetworkTable PIDInfo, pidTuningPVs; // , camInfo;
  NetworkTableEntry
  // Declare Shooter PID tuning entries
  shootPEntry, shootIEntry, shootDEntry, shootFFmEntry, shootFFbEntry, bypassShooterPIDEntry,
      // Declare Turret PID tuning entries
      turretPEntry, turretIEntry, turretDEntry, turretFFmEntry, turretFFbEntry,
      // Declare information entries
      shooterSpeedEntry, shooterCurrSpeedEntry, shooterAtSpeedEntry, turretEncoderEntry, turretLastTargetEntry,
      turretLastTargetOffsetEntry, realDistanceEntry, targetShooterSpeed,
      // Declare targetting entries
      onTargetEntry; // , validTargetEntry, pitchEntry, latencyEntry, targetPoseEntry, distanceEntry,
                     // yawEntry, onTargetEntry;

  PID shootingMotorPID, turretPositionPID;
  Boolean bypassShooterPID;
  double shootP, shootI, shootD, shootFFm, shootFFb, shooterRawSpeed;
  double turretP, turretI, turretD;
  double shooterSetSpeed;

  double turretOffset;
  private double dynamicAimOffset = 0;
  private double dynamicSpeedOffset = 0;

  double yaw, pitch, area, skew, latency, distance;
  public Boolean validTarget;
  public PhotonTrackedTarget photonTarget;

  CANSparkMax turretMotor, shootingMotorLeft, shootingMotorRight;
  SparkMaxRelativeEncoder shooterEncoder, turretEncoder;

  TalonSRX shootPreload;

  PowerDistribution powerDistribution;

  RobotDriveSubsystem driveSubsystem;

  Boolean shotInProgress;

  PhotonCamera targettingCam;

  public TurretSubsystem(RobotDriveSubsystem driveSub) {
    this.driveSubsystem = driveSub;
    this.distanceAverage = new MovingAverage(10);
    this.shooterSpeedAverage = new MovingAverage(5);
    this.shotInProgress = true;
    this.targettingCam = new PhotonCamera("USB_Camera-B4.09.24.1");
    this.powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    this.powerDistribution.setSwitchableChannel(false);

    // setupLogging();
    setupNetworkTables();
    setupMotorsAndEncoders();
    setupPIDControllers();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getValues();
    updateShooterPIDValues();
    updateTurretPIDValues();
    updateNetworkTables();
  }

  public void runShooterRaw(double speed) {
    this.targetShooterSpeed.setDouble(speed * 5420);
    shootingMotorLeft.set(speed);
    shootingMotorRight.set(-speed);
  }

  // Directly related to shooter
  double prevSetpoint = 0;

  public void runShooterAtSpeed(double speed) {
    speed = -speed;
    double targetVel = speed * 5415;
    this.targetShooterSpeed.setDouble(targetVel);
    this.shootingMotorPID.updateSetpoint(targetVel);

    double delta = this.prevSetpoint - targetVel;
    if (delta > 100 || delta < -100) {
      this.shootingMotorPID.resetIntegral();
    }

    double workingSpeed = this.shootingMotorPID.run(this.shooterEncoder.getVelocity());
    if (workingSpeed < 0) {
      workingSpeed = 0;
    }

    if (speed <= 0) {
      workingSpeed = speed;
    }

    if (this.bypassShooterPID) {
      workingSpeed = speed; // Bypass PID if it's going nuts or speed = 0
    }

    shootingMotorLeft.set(-workingSpeed);
    shootingMotorRight.set(workingSpeed);

    Logging.debug("FF set: " + targetVel + ", FF out: " + this.shooterEncoder.getVelocity(), "FFTuning");

    Logging.debug("Input:" + workingSpeed, "shooterPID");
    Logging.debug("Shooter:" + this.shooterEncoder.getVelocity(), "shooterPID");
    Logging.debug("Target:" + targetVel, "shooterPID");

    String recordsString = String.format("%f,%f,%f,%f,%f", (double) System.currentTimeMillis(), (double) speed,
        (double) targetVel, (double) workingSpeed, (double) this.shooterEncoder.getVelocity());

    this.shooterLog(recordsString);

    this.prevSetpoint = targetVel;
  }

  public boolean shooterAtSpeed() {
    double setpoint = this.shootingMotorPID.getSetpoint();
    double processingVar = this.shooterEncoder.getVelocity();
    this.shooterSpeedAverage.push(processingVar);
    // double setpointAverage = this.shooterSpeedAverage.getAverage();
    // boolean averageAtSpeed = setpointAverage - Constants.shooterMarginOfError <
    // processingVar
    // && processingVar < setpointAverage + Constants.shooterMarginOfError;
    boolean atSpeed = setpoint - Constants.shooterMarginOfError < processingVar
        && processingVar < setpoint + Constants.shooterMarginOfError;
    return atSpeed;
  }

  public boolean onTarget() {
    double setpoint = 0;
    double adjustmentAngle = Constants.staticAimOffset + this.dynamicAimOffset;
    double processingVar = this.yaw + adjustmentAngle;
    boolean onTarget = setpoint - Constants.turretMarginOfError < processingVar
        && processingVar < setpoint + Constants.turretMarginOfError;
    return onTarget;
  }

  public double getSpeedFromDist(double dist) {
    double speed = Constants.speedkM * dist + Constants.speedkB;
    return speed;
  }

  boolean distanceLocked = false;
  boolean savedDistanceBool = false;
  double savedDistance = -1;

  public void lockDistance() {
    this.distanceLocked = true;
  }

  public void unlockDistance() {
    this.distanceLocked = false;
  }

  public double getDistance() {
    getDistanceVal();
    return this.distance;
  }

  private void getDistanceVal() {
    if (this.targettingEnabled) {
      this.distanceAverage
          .push(PhotonUtils.calculateDistanceToTargetMeters(
              Constants.cameraHightMeters,
              Constants.targetHightMeters,
              Units.degreesToRadians(Constants.cameraPitchDegrees),
              Units.degreesToRadians(this.pitch)) + 0.45);
    }
    double measuredDist = this.distanceAverage.getAverage();
    double dist;
    this.distanceLocked = false;
    if (this.distanceLocked) {
      if (!this.savedDistanceBool) {
        this.savedDistance = measuredDist;
        this.savedDistanceBool = true;
      }
      dist = savedDistance;
    } else {
      dist = measuredDist;
      this.savedDistanceBool = false;
    }

    this.distance = dist;
  }

  public double getManualSpeed() {
    return this.shooterSetSpeed;
  }

  public void feedShooter() {
    this.shootPreload.set(TalonSRXControlMode.PercentOutput, 1);
  }

  public void feedShooterRaw(double speed) {
    this.shootPreload.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stopFeeder() {
    this.shootPreload.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void resetShooterIntegral() {
    this.shootingMotorPID.resetIntegral();
  }

  public void updateShooterPIDValues() {
    this.shootP = this.shootPEntry.getDouble(Constants.shooterkP);
    this.shootI = this.shootIEntry.getDouble(Constants.shooterkI);
    this.shootD = this.shootDEntry.getDouble(Constants.shooterkD);
    this.shootFFm = this.shootFFmEntry.getDouble(Constants.shooterkFFb);
    this.shootFFb = this.shootFFbEntry.getDouble(Constants.shooterkFFm);
    this.bypassShooterPID = this.bypassShooterPIDEntry.getBoolean(Constants.bypassShooterPID);

    this.shootingMotorPID.updateP(this.shootP);
    this.shootingMotorPID.updateI(this.shootI);
    this.shootingMotorPID.updateD(this.shootD);
    this.shootingMotorPID.getFF().updateValues(new double[] { this.shootFFm, this.shootFFb });

    Logging.debug("\nShooter PID Values: kP: " + this.shootingMotorPID.getP() + "\nkI: " + this.shootingMotorPID.getI()
        + "\nkD: " + this.shootingMotorPID.getD() + "\nkFFm: " + this.shootingMotorPID.getFF().getValues()[0]
        + "\nkFFb: " + this.shootingMotorPID.getFF().getValues()[1], "shootingPID");
  }

  public void setTurretTarget(double angle) {
    this.turretPositionPID.updateSetpoint(angle);
  }

  // Directly related to turret
  public void turnToAngle(double angle) {

    if (Math.abs(angle) > Constants.turretSpinLimit) {
      angle = Math.copySign(Constants.turretSpinLimit, angle);
    }
    this.setTurretTarget(angle);
    double speed = this.turretPositionPID.run(this.getTurretPosRaw());
    this.turretMotor.set(speed);

  }

  public void turnToAngle(double angle, double maxSpeed) {
    this.setTurretTarget(angle);
    double speed = this.turretPositionPID.run(this.getTurretPosRaw());
    if (Math.abs(speed) > maxSpeed) {
      double sign = speed > 0 ? 1 : -1;
      speed = maxSpeed * sign;
    }
    this.turretMotor.set(speed);
  }

  public void turnRaw(double speed) {
    this.turretMotor.set(speed);
  }

  // private double getDistanceToInner(double angle, double distance, double
  // targetDelta) {
  // return Math.sqrt(Math.pow(targetDelta, 2) + Math.pow(distance, 2) -
  // targetDelta * distance * Math.cos(angle));
  // }

  // private double getAngleOffset(double angle, double distance, double
  // targetDelta) {
  // return Math.asin(targetDelta / distance * Math.sin(angle));
  // }

  boolean pastLimit = false;
  // private double lastTargetPos = 0;
  // private double lastValidTurretPos = 0;
  private boolean targettingEnabled = false;

  public void enableTargetting(boolean enable) {
    if (enable) {
      this.powerDistribution.setSwitchableChannel(true);
      this.targettingEnabled = true;
    } else {
      this.powerDistribution.setSwitchableChannel(false);
      this.targettingEnabled = false;
    }
  }

  public void setDynamicOffset(double offset) {
    this.dynamicAimOffset = offset;
  }

  public double getDynamicOffset() {
    return this.dynamicAimOffset;
  }

  public void setSpeedOffset(double offset) {
    this.dynamicSpeedOffset = offset;
  }

  public double getSpeedOffselt() {
    return this.dynamicSpeedOffset;
  }

  public double getShooterSpeedRPM() {
    return this.shooterEncoder.getVelocity();
  }

  public double getShooterSpeedFPS() {
    return Utilities.shooterRPMtoFPS(this.shooterEncoder.getVelocity());
  }

  // public double getTargetAngle() {
  // return this.poseAngle;
  // }

  private double storedTurretAngle = 0;
  private boolean turretLocked = false;

  public void lockTurret() {
    if (!this.turretLocked) {
      this.turretLocked = true;
      this.storedTurretAngle = this.getTurretPos();
    }
  }

  public void unlockTurret() {
    this.turretLocked = false;
    this.storedTurretAngle = this.getTurretPos();
  }

  public void turnToTarget() {
    turnToTarget(1);
  }

  public void turnToTarget(double maxspeed) {
    if (!this.turretLocked) {
      setTurretTarget(0);
      // Logging.debug("Turret limit: " + Constants.turretSpinLimit + "\nTurret
      // Position: " + this.getTurretPos()
      // + "\nTurret Past Limit: " + this.pastLimit + "\nValid Target: " +
      // this.validTarget, "turretLimits");
      if (!this.pastLimit && this.validTarget) {
        this.pastLimit = Math.abs(this.getTurretPosRaw()) > Constants.turretSpinLimit;
        double adjustAngle = Constants.staticAimOffset + this.dynamicAimOffset;
        double processingVar = this.yaw + adjustAngle;
        double speed = this.turretPositionPID.run(processingVar);

        // Logging.debug("Aiming PID Output Value: " + speed + "\nAiming PV: " +
        // processingVar, "aimingPID");

        if (Math.abs(speed) > maxspeed) {
          speed = Math.copySign(maxspeed, speed);
        }

        this.turnRaw(speed);
        // this.lastTargetPos = this.lastValidTurretPos = this.getTurretPos();
      }
    } else {
      this.turnToAngle(this.storedTurretAngle);
    }

  }

  private double getTurretPosRaw() {
    double turretPosition = (this.turretEncoder.getPosition() / Constants.turretToMoterRatio) * 360;
    return turretPosition;
  }

  public double getTurretPos() {
    double turretPositionDegrees = (this.getTurretPosRaw()) % 360;
    double turretPositionClamped = this.clamp(turretPositionDegrees);
    return turretPositionClamped;
  }

  public void updateTurretPIDValues() {
    this.turretP = this.turretPEntry.getDouble(Constants.turretkP);
    this.turretI = this.turretIEntry.getDouble(Constants.turretkI);
    this.turretD = this.turretDEntry.getDouble(Constants.turretkD);

    this.turretPositionPID.updateP(this.turretP);
    this.turretPositionPID.updateI(this.turretI);
    this.turretPositionPID.updateD(this.turretD);

    Logging.debug("Turret PID Values: kP: " + this.turretPositionPID.getP() + " kI: " + this.turretPositionPID.getI()
        + " kD: " + this.turretPositionPID.getD(), "turretPID");
  }

  // Related to everything
  private void updateNetworkTables() {
    this.shooterCurrSpeedEntry.setDouble(this.shooterEncoder.getVelocity());
    this.turretEncoderEntry.setDouble(this.getTurretPos()); // 360 degrees = 100
    // this.distanceEntry.setDouble(this.distance);
    this.shooterAtSpeedEntry.setBoolean(this.shooterAtSpeed());
  }

  private void setupMotorsAndEncoders() {
    // Set up Motors
    this.turretMotor = new CANSparkMax(Constants.turretSparkMax, MotorType.kBrushless);
    this.turretMotor.setSmartCurrentLimit(Constants.miniNeoSafeAmps);
    this.turretMotor.setIdleMode(IdleMode.kCoast);
    this.turretMotor.set(0);

    this.shootingMotorLeft = new CANSparkMax(Constants.shooterSparkMaxLeft, MotorType.kBrushless);
    this.shootingMotorLeft.setSmartCurrentLimit(Constants.neoSafeAmps);
    this.shootingMotorLeft.setIdleMode(IdleMode.kCoast);

    this.shootingMotorRight = new CANSparkMax(Constants.shooterSparkMaxRight, MotorType.kBrushless);
    this.shootingMotorRight.setSmartCurrentLimit(Constants.neoSafeAmps);
    this.shootingMotorRight.setIdleMode(IdleMode.kCoast);

    this.shootPreload = new TalonSRX(Constants.shooterPreloadTalonSRX);

    // Set up Encoders
    this.turretEncoder = (SparkMaxRelativeEncoder) this.turretMotor.getEncoder();
    this.shooterEncoder = (SparkMaxRelativeEncoder) this.shootingMotorRight.getEncoder();

    this.turretEncoder.setPosition(0.25 * Constants.turretToMoterRatio);
    this.shooterEncoder.setPosition(0);
  }

  private void setupPIDControllers() {
    // Define PID controllers
    this.shootingMotorPID = new PID.PIDBuilder(0, Constants.shooterkP, Constants.shooterkI, Constants.shooterkD)
        .FF(new LinearFF(Constants.shooterkFFm, Constants.shooterkFFb)).MinIntegral(-Constants.shooterSZone)
        .MaxIntegral(Constants.shooterSZone).IZone(Constants.shooterIZone).Name("shooting").build();
    this.turretPositionPID = new PID.PIDBuilder(0, Constants.turretkP, Constants.turretkI, Constants.turretkD)
        .MinIntegral(-Constants.turretIZone).MaxIntegral(Constants.turretIZone).Name("turretPosition").build();

    /*
     * this.shootingMotorPID = new PID(0, // Default setpoint Constants.shooterkP,
     * Constants.shooterkI, Constants.shooterkD, new LinearFF(Constants.shooterkFFm,
     * Constants.shooterkFFb), Constants.shooterIZone, -Constants.shooterSZone,
     * Constants.shooterSZone);
     * 
     * this.turretPositionPID = new PID(0, // Default setpoint Constants.turretkP,
     * Constants.turretkI, Constants.turretkD, new ConstantFF(0.0),
     * -Constants.turretIZone, Constants.turretIZone);
     */
  }

  private void setupNetworkTables() {
    this.table = NetworkTableInstance.getDefault();
    this.PIDInfo = this.table.getTable("PID");
    this.pidTuningPVs = this.table.getTable("pidTuningPVs");
    // this.camInfo =
    // this.table.getTable("photonvision").getSubTable("USB_Camera-B4.09.24.1");

    // Define Monitor entries
    this.shooterSpeedEntry = this.PIDInfo.getEntry("shootTargetSpeed");
    this.shooterCurrSpeedEntry = this.pidTuningPVs.getEntry("Shooter Speed");
    this.targetShooterSpeed = this.pidTuningPVs.getEntry("shooterTargetSpeed");
    this.shooterAtSpeedEntry = this.pidTuningPVs.getEntry("Shooter at Speed");
    this.turretEncoderEntry = this.pidTuningPVs.getEntry("Turret Position");

    // Define Shooter PID entries
    this.shootPEntry = this.PIDInfo.getEntry("shootkP");
    this.shootIEntry = this.PIDInfo.getEntry("shootkI");
    this.shootDEntry = this.PIDInfo.getEntry("shootkD");
    this.shootFFmEntry = this.PIDInfo.getEntry("shootkFFm");
    this.shootFFbEntry = this.PIDInfo.getEntry("shootkFFb");
    this.bypassShooterPIDEntry = this.PIDInfo.getEntry("bypassShooterPID");

    // Define turret PID entries
    this.turretPEntry = this.PIDInfo.getEntry("turretkP");
    this.turretIEntry = this.PIDInfo.getEntry("turretkI");
    this.turretDEntry = this.PIDInfo.getEntry("turretkD");

    // Define other entries
    this.turretLastTargetEntry = this.pidTuningPVs.getEntry("Last Known Target Pos");
    this.turretLastTargetOffsetEntry = this.pidTuningPVs.getEntry("Offset From Last Known Target Pos");

    // Set default values
    this.shootPEntry.setDouble(Constants.shooterkP);
    this.shootIEntry.setDouble(Constants.shooterkI);
    this.shootDEntry.setDouble(Constants.shooterkD);
    this.shootFFmEntry.setDouble(Constants.shooterkFFm);
    this.shootFFbEntry.setDouble(Constants.shooterkFFb);
    this.bypassShooterPIDEntry.setBoolean(Constants.bypassShooterPID);
    this.shooterCurrSpeedEntry.setDouble(0);
    this.shooterAtSpeedEntry.setBoolean(false);
    this.shooterSpeedEntry.setDouble(Constants.shooterDefaultSpeed);
    this.targetShooterSpeed.setDouble(0);

    this.turretPEntry.setDouble(Constants.turretkP);
    this.turretIEntry.setDouble(Constants.turretkI);
    this.turretDEntry.setDouble(Constants.turretkD);
  }

  private void getValues() {
    if (!this.shotInProgress) {

      try {
        this.validTarget = this.targettingCam.getLatestResult().hasTargets();
        if (this.validTarget) {
          this.photonTarget = this.targettingCam.getLatestResult().getBestTarget();
          if (this.photonTarget != null) {
            this.yaw = this.photonTarget.getYaw();
            this.pitch = this.photonTarget.getPitch();
            this.area = this.photonTarget.getArea();
            this.skew = this.photonTarget.getSkew();
          }
          getDistanceVal();
        } else {
          this.photonTarget = null;
        }
      } catch (NullPointerException e) {

      }
    }
    this.shooterSetSpeed = this.shooterSpeedEntry.getDouble(Constants.shooterDefaultSpeed);
    this.pidTuningPVs.getEntry("distanceToTarget").setDouble(this.distance);
  }

  public void resetTurretEncoder() {
    this.turretEncoder.setPosition(0);
  }

  private double clamp(double val) {
    if (val > 180) {
      val -= 360;
    } else if (val < -180) {
      val += 360;
    }
    return val;
  }

  // private void setupLogging() {
  // Path deployPath = Filesystem.getDeployDirectory().toPath();
  // File shooterLoggingFile = new
  // File(deployPath.resolve("csv/Shooter.csv").toString());
  // try {
  // this.shooterLogging = new CSVWriter(new FileWriter(shooterLoggingFile));
  // this.shooterWriterActive = true;
  // } catch (IOException e) {
  // System.err.println("Failed to open shooterLogging because:");
  // System.err.println(e.toString());
  // this.shooterWriterActive = false;
  // }
  // this.shooterLog("Time,Input%,TargetRPM,Output%,CurrentRPM");
  // }

  public void enableShooterLogging(boolean enable) {
    this.enableShooterLogging = enable;
  }

  public void closeShooterLogging() {
    if (this.shooterWriterActive) {
      // try {
      // // this.shooterLogging.close();
      // } catch (IOException e) {

      // }
    }
    this.shooterWriterActive = false;
  }

  private void shooterLog(String recordsString) {
    // String[] records = recordsString.split(",");
    if (this.shooterWriterActive && this.enableShooterLogging) {
      // this.shooterLogging.writeNext(records);
    }
  }

  public void stop() {
    this.shootPreload.set(TalonSRXControlMode.PercentOutput, 0);
    this.shootingMotorLeft.set(0);
    this.shootingMotorRight.set(0);
    this.turretMotor.set(0);
    this.powerDistribution.setSwitchableChannel(false);
  }

  public void setShotInProgress(boolean ShotInProgress) {
    this.shotInProgress = ShotInProgress;
  }

  public boolean getShotInProgress() {
    return this.shotInProgress;
  }

}
