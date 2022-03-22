/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlingSubsystem extends SubsystemBase {
  private NetworkTable bling;
  private NetworkTableEntry chassisSelect;
  private NetworkTableInstance table;
  private SendableChooser<String> chassisSelector;
  private boolean ignoreSelector = false;

  public BlingSubsystem() {
    setupNetworkTables();
  }

  @Override
  public void periodic() {
    if (!this.ignoreSelector) {
      this.chassisSelect.setString(this.chassisSelector.getSelected());
    }
  }

  private void setupNetworkTables() {
    this.table = NetworkTableInstance.getDefault();
    this.bling = this.table.getTable("Bling");
    this.chassisSelect = this.bling.getEntry("blingSelect");
    this.chassisSelector = new SendableChooser<>();

    this.chassisSelector.addOption("Fancy Idle Mode", "fancyidle");
    this.chassisSelector.addOption("Red vs Blue", "fight");
    this.chassisSelector.setDefaultOption("Idle Mode", "idle");
    this.chassisSelector.addOption("Bling Test", "grade");
    this.chassisSelector.addOption("Disable Bling", "off");
    Shuffleboard.getTab("Bling").add(this.chassisSelector);
    
  }

  public void setBling(String blingSel) {
    this.chassisSelect.setString(blingSel);
  }

  public void setSelectorIgnore(boolean ignore) {
    this.ignoreSelector = ignore;
  }
}
