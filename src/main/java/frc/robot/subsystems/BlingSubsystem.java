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
  private NetworkTableEntry blingSelect;
  private NetworkTableInstance table;
  private SendableChooser<String> blingSelector;
  private boolean ignoreSelector = false;

  public BlingSubsystem() {
    setupNetworkTables();
  }

  @Override
  public void periodic() {
    if (!this.ignoreSelector) {
      this.blingSelect.setString(this.blingSelector.getSelected());
    }
  }

  private void setupNetworkTables() {
    this.table = NetworkTableInstance.getDefault();
    this.bling = this.table.getTable("Bling");
    this.blingSelect = this.bling.getEntry("blingSelect");
    this.blingSelector = new SendableChooser<>();

    this.blingSelector.setDefaultOption("Fancy Idle Mode", "fancyidle");
    this.blingSelector.addOption("Red vs Blue", "fight");
    this.blingSelector.addOption("Idle Mode", "idle");
    this.blingSelector.addOption("Bling Test", "grade");
    this.blingSelector.addOption("Disable Bling", "off");

    Shuffleboard.getTab("Bling").add(this.blingSelector);
  }

  public void setBling(String blingSel) {
    this.blingSelect.setString(blingSel);
  }

  public void setSelectorIgnore(boolean ignore) {
    this.ignoreSelector = ignore;
  }
}
