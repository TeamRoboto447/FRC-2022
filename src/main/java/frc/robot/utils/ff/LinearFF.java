/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.ff;

/**
 * Add your docs here.
 */
public class LinearFF extends FFbase {
    double
        M,
        B;
    public LinearFF ( double kM, double kB){
        this.M = kM;
        this.B = kB;
    }
    @Override
    public double getFF(double setpoint, double processingVar, double iterTime){
       return this.M * setpoint + this.B;
    }
    @Override
    public double[] getValues(){
        return new double[] {this.M, this.B};
    }
    @Override
    public void updateValues(double[] values){
        this.M = values[0];
        this.B = values[1];
    }
    @Override
    public String getValsAsString(){
        return String.format("FFType: Linear, FFm: %f, FFb: %f",this.M,this.B);
    }
    public double getM(){
        return this.M;
    }
    public double getB(){
        return this.B;
    }
}
