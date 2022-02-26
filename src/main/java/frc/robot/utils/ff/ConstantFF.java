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
public class ConstantFF extends FFbase {
    double v;
    public ConstantFF(double kV){
        this.v = kV;
    }
    @Override
    public double getFF(double setpoint, double processingVar, double iterTime){
        if (setpoint > processingVar){
            return this.v;
        } else if (setpoint < processingVar) {
            return - this.v;
        } else {
            return 0.0;
        }
    }
    @Override
    public void updateValues(double[] values){
        this.v = values[0];
    }
    @Override
    public String getValsAsString(){
        return  String.format("FFType: Constant, FFv: %f",this.v);
    }

}
