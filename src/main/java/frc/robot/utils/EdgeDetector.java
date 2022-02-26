/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Add your docs here.
 */
public class EdgeDetector {
    boolean lastState;
    boolean fallingEdge;
    public EdgeDetector(boolean fallingEdge){
        this.fallingEdge = fallingEdge;
        this.lastState = !fallingEdge;
    }

    public boolean detect(boolean input){
        if (this.fallingEdge && this.lastState && !input){
            this.lastState = input;
            return true;
        } else if (!this.fallingEdge && !this.lastState && input){
            this.lastState = input;
            return true;
        }
        this.lastState = input;
        return false;
    }
}
