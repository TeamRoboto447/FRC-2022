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
public class MovingAverage {
    double[] values;
    int index = 0;
    boolean full = false;
    int maxSize;

    public MovingAverage (int maxSize){
        this.values = new double[maxSize];
        this.maxSize = maxSize;
    }

    public void push(double value){
        this.values[this.index] = value;
        if (this.index == this.maxSize - 1){
            this.index = 0;
            this.full = true;
        } else {
            this.index ++;
        }
    }

    public double getAverage(){
        double sum = 0;
        int sampleSize;
        
        if (this.full){
            sampleSize = this.maxSize;
        } else {
            sampleSize = this.index + 1; 
        }
        for (int i = 0; i < sampleSize; i++){
            sum += this.values[i];
        }
        return sum / sampleSize;
    }

    public void reset(){
        this.index = 0;
        this.full = false;
    }
}
