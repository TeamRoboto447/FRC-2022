/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
/**
 * Add your docs here.
 */
public class Logging {
    static NetworkTableEntry subsystemToDebug;
    public static void errors(String message, String name){
        if (Constants.loggingLevel >= 0){
            loggingBase(message, name);
        }
    }

    public static void warnings(String message, String name){
        if (Constants.loggingLevel >= 1){
            loggingBase(message, name);
        }
    }

    public static void debug(String message, String name) {
        if (Constants.loggingLevel >= 2){
            loggingBase(message, name);
        }
    }

    public static void info(String message, String name){
        if (Constants.loggingLevel >= 3){
            loggingBase(message, name);
        }
    }

    private static void loggingBase(String message, String name){
        if (subsystemToDebug.getString("errored").contains("everything;")){
            System.out.println(message);
        } else if ((";"+subsystemToDebug.getString("errored")).contains(";"+name+";")){
            System.out.println(message);
        }
    }

    public static void init() {
        subsystemToDebug = NetworkTableInstance.getDefault().getTable("logging").getEntry("subsysToLog");
        subsystemToDebug.setString("");
    }
}
