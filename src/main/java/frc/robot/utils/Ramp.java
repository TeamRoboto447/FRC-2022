/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import java.lang.Math;

/**
 * Add your docs here.
 */
public class Ramp {
    double 
        maxChangePerSecond,
        lastValue,
        maxTimeDelta;
    boolean enabled;
    Timer timeSinceLast;
    public Ramp(double timeToMax, double kMaxTimeDelta, boolean kEnabled){
        this.maxChangePerSecond = 1 / timeToMax;
        this.lastValue = 0;
        this.timeSinceLast = new Timer();
        this.timeSinceLast.reset();
        this.timeSinceLast.start();
        this.maxTimeDelta = kMaxTimeDelta;
        this.enabled = kEnabled;
    }
    public void setEnable(boolean state){
        this.enabled = state;
    }
    public double run(double value){
        double timeDelta = this.timeSinceLast.get();
        this.timeSinceLast.reset();
        this.timeSinceLast.start();
        if (!this.enabled){
            this.lastValue = 0.0;
            return value;
        }
        if (timeDelta > this.maxTimeDelta){
            timeDelta = this.maxTimeDelta;
        }
        if (Math.abs(value) <= Math.abs(this.lastValue) && value * this.lastValue > 0){
            this.lastValue = value;
            return value;
        }else if (Math.abs(value - this.lastValue) > this.maxChangePerSecond * timeDelta){
            this.lastValue += Math.copySign(this.maxChangePerSecond * timeDelta, value-this.lastValue);
            return this.lastValue;
        } else {
            this.lastValue = value;
            return value;
        }
    }
}
