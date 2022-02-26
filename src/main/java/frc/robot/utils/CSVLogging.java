/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

//import java.io.File;
// import java.io.FileWriter;
// import java.io.IOException;
// import java.nio.file.Path;

// import com.opencsv.CSVWriter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class CSVLogging {
    // private CSVWriter file;
    // private Boolean open = false;
    private String name;
    private NetworkTableEntry subSystemToLogCSV;
    private NetworkTableEntry subSystemToLog;
    private int level;
    private String[] headers;

    public CSVLogging(String[] headers, String name, int level) {
        this.name = name;
        this.level = level;
        this.headers = headers;
        //this.open();
        this.subSystemToLogCSV = NetworkTableInstance.getDefault().getTable("logging").getEntry("subsysToLogCSV");
        this.subSystemToLogCSV.setString("");
        this.subSystemToLog = NetworkTableInstance.getDefault().getTable("logging").getEntry("subsysToLog");
        this.subSystemToLog.setString("");
    }

    // public boolean open() {
    //     try {
    //         Path deployPath = Filesystem.getDeployDirectory().toPath();
    //         Path filePath = deployPath.resolve("csv/" + this.name + ".csv");
    //         this.file = new CSVWriter(new FileWriter(filePath.toString()));
    //         this.open = true;
    //     } catch (IOException e) {
    //         this.open = false;
    //         System.err.println("Could not start csv logging. Exception below: \n" + e.toString());
    //     }
    //     if (this.open) {
    //         this.file.writeNext(this.headers);
    //     }
    //     return this.open();
    // }

    // public void close() {
    //     try {
    //         this.file.close();
    //     } catch (IOException e) {

    //     }
    // }

    // public void logCSV(String[] row) {
    //     if (this.level <= Constants.loggingLevel && Constants.enableCSVLogging) {
    //         if (this.subSystemToLogCSV.getString("errored").contains("everything;")) {
    //             if (!this.open && !this.open()) {
    //                 return;
    //             }
    //             this.file.writeNext(row);
    //         } else if ((";" + this.subSystemToLogCSV.getString("errored")).contains(";" + this.name + ";")) {
    //             if (!this.open && !this.open()) {
    //                 return;
    //             }
    //             this.file.writeNext(row);
    //         } else if (this.open) {
    //             this.close();
    //         }
    //     }
    // }

    public void logToStdOut(String[] row) {
        if (this.level <= Constants.loggingLevel) {
            if (this.subSystemToLog.getString("errored").contains("everything;")) {
                System.out.println(this.appendHeaders(row));
            } else if ((";" + this.subSystemToLog.getString("errored")).contains(";" + this.name + ";")) {
                System.out.println(this.appendHeaders(row));
            }
        }
    }

    public void logBoth(String[] row) {
        this.logToStdOut(row);
        // this.logCSV(row);
    }

    public void logBoth(double[] row) {
        String[] rowOut = new String[row.length];
        for (int i = 0; i < row.length; i++) {
            rowOut[i] = "" + row[i];
        }
        this.logBoth(rowOut);
    }

    private String appendHeaders(String[] row) {
        String out = "";
        for (int i = 0; i < row.length; i++) {
            out += String.format("%s: %s, ", this.headers[i], row[i]);
        }
        return out.substring(0, out.length() - 2);
    }
}
