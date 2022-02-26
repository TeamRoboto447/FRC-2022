/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.ff;

import frc.robot.Constants;

/**
 * This is a very specific Feed Forward class. It specifically looks at the
 * speed of the turret the speed of the movement of the target on the turret
 * and uses that information to determin how fast to move the turret.
 * This is technically not only Feed Forward but also Feed Back, but that is
 * only important for Aden. 
 */
public class TurretFF extends FFbase {
    final double setpointToRotations = 1/360;
    private double 
        encoderSpeed = 0.0,
        M,
        B;
    double lastSetpoint;
    
    public TurretFF( double kM, double kB ){
        this.M = kM;
        this.B = kB;
    }

    @Override
    public double getFF(double setpoint, double processingVar, double iterTime){
        double lSetpoint = this.lastSetpoint;
        this.lastSetpoint = setpoint;
        double turretAngularVelosity = this.getTurretSpeed(); 
        double imageAngularVelosity = (setpoint - lSetpoint) * this.setpointToRotations / iterTime;
        double targetAngularVelosity = turretAngularVelosity + imageAngularVelosity;
        double targetAngularVelosityMotor = targetAngularVelosity * Constants.turretToMoterRatio;
        return this.M * targetAngularVelosityMotor + this.B;
    }
    @Override
    public String getValsAsString(){
        return String.format("FFType: Turret, FFm: %f, FFb: %f", this.M, this.B);
    }
    public void setEncoderSpeed(double encSpeed){
        //This is expected to be in rotations
        this.encoderSpeed = encSpeed;
    }
    private double getTurretSpeed(){
        return this.encoderSpeed / Constants.turretToMoterRatio;
    }
    
}
