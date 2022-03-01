/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoSink;
// import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
// import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Logging;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private PneumaticHub pneumaticHub;
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  // private UsbCamera camera0;
  // private VideoSink camServer;

  private NetworkTable pidTuningPVs;
  private NetworkTableInstance table;
  private NetworkTableEntry timeEntry;

  // private PowerDistributionPanel pdp;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    this.robotContainer = new RobotContainer();
    this.pneumaticHub = new PneumaticHub();
    this.pneumaticHub.enableCompressorDigital();
    PortForwarder.add(5800, "photonvision.local", 5800);

    // this.camera0 = CameraServer.startAutomaticCapture(0);
    // this.camera0.setResolution(160, 120);
    // this.camera0.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    // this.camServer = CameraServer.getServer();

    Logging.init();

    this.table = NetworkTableInstance.getDefault();
    this.pidTuningPVs = table.getTable("pidTuningPVs");
    this.timeEntry = pidTuningPVs.getEntry("timeMS");

    // this.pdp = new PowerDistributionPanel();
    // Shuffleboard.getTab("PDP").add("PDP", this.pdp).withWidget(BuiltInWidgets.kPowerDistributionPanel);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    this.timeEntry.setDouble(System.currentTimeMillis());
  }

  private void setRobotFront() {
    // this.camServer.setSource(this.camera0);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    // robotContainer.testDriveSubsystem.enableLogging(false);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    robotContainer.driveSubsystem.enableLogging(false);
    robotContainer.indexerSubsystem.lowerIntake();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.driveSubsystem.enableLogging(false);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    setRobotFront();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // robotContainer.driveSubsystem.closeCSVs();
    // robotContainer.turretSubsystem.closeShooterLogging();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
