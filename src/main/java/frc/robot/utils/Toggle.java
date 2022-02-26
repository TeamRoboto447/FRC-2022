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

public class Toggle {
    private boolean state;
    private boolean buttonPressed = false;

    public Toggle(boolean defaultState){
        this.state = defaultState;
    }
    
    public boolean runToggle(boolean button){
        if (button && !this.buttonPressed){
            toggle();
            this.buttonPressed = true;
        } else if(!button && this.buttonPressed) {
            this.buttonPressed = false;
            return true;
        }
        return false;
    }

    public boolean getState(){
        return this.state;
    }

    public boolean setState(boolean state){
        this.state = state;
        return this.state;
    }
    
    private boolean toggle() {
        this.state = !this.state;
        return this.state;
    }
}
