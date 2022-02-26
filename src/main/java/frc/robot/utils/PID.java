/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;
import frc.robot.utils.ff.FFbase;
/*
 * Add your docs here.
 */
public class PID {
    private double 
        integral, 
        previousError, 
        P, 
        I, 
        D,
        setpoint,
        maxInt,
        minInt,
        izone;
    private Timer 
        iterTime,
        integralTimer;
    private FFbase FF;
    private String name = "";
    private CSVLogging 
        valueLogging,
        setpointLogging;
    public PID (PIDBuilder builder){
        this.P = builder.P;
        this.I = builder.I;
        this.D = builder.D;
        this.FF = builder.kFF;
        this.setpoint = builder.setpoint;
        this.minInt = builder.minInt;
        this.maxInt = builder.maxInt;
        this.izone = builder.izone;
        this.name = builder.name;  

        this.valueLogging = new CSVLogging(new String[]{"time","P","I","D","FF","total"},  builder.name + "PIDOutVals", Constants.debug);
        this.setpointLogging = new CSVLogging(new String[]{"time","setpoint","pv"}, builder.name + "PIDSetpoint", Constants.debug);
        this.iterTime = new Timer();
        this.iterTime.reset();
        this.iterTime.stop();
        this.integralTimer = new Timer();
        this.integralTimer.reset();
    }

    public double run(double processingVar) {
        double iterTime = this.iterTime.get();
        this.iterTime.start();
        this.iterTime.reset();
        
        if (this.integralTimer.get()>=Constants.shooterPidIntegralResetTime){
            this.resetIntegral();
            this.integralTimer.reset();
        }

        final double error = this.setpoint - processingVar;
        if ( Math.abs(error) <= this.izone || this.izone==0){
            this.integral += (error * iterTime);
        }
        if (this.minInt != 0 && this.integral<this.minInt){
            this.integral = this.minInt;
        } else if (this.maxInt != 0 && this.integral>this.maxInt){
            this.integral = this.maxInt;
        }

        final double derivitive = (error - this.previousError) / iterTime;
        final double feedforward = this.FF.getFF(this.setpoint,processingVar,iterTime);
        
        double total = this.P * error + this.I * this.integral + this.D * derivitive + feedforward;
        double time = System.currentTimeMillis() / 1000;

        Logging.info("PID time: " + iterTime, this.name + "PIDtime");
        //Logging.info(String.format("P: %f, I: %f, D: %f, FF: %f",this.P * error, this.I * this.integral, this.D * derivitive,feedforward), this.name + "PIDOutVals");
        this.valueLogging.logBoth(new double[]{time, this.P * error, this.I * this.integral, this.D * derivitive, feedforward, total});
        this.setpointLogging.logBoth(new double[]{time, this.setpoint, processingVar});
        Logging.info(String.format("P: %f, I: %f, D: %f, %s",this.P, this.I, this.D, this.FF.getValsAsString()), this.name + "PIDVals");

        this.previousError = error;
        return total;
    }
    
    public void resetIntegral() {
        Logging.debug("Integral reset prev val: "+this.integral, this.name+"PIDIntegralReset");
        this.integral = 0;
        this.iterTime.reset();
        this.iterTime.stop();
        Logging.debug("Integral reset after val: "+this.integral, this.name+"PIDIntegralReset");
    } 
    
    public void updateP(double kP){ this.P = kP; }
    public void updateI(double kI){ this.I = kI; }
    public void updateD(double kD){ this.D = kD; }
    public void updateSetpoint(double kSetpoint){ this.setpoint = kSetpoint; }
    public void updateFF(FFbase kFF){ this.FF = kFF; }
    
    public double getP() { return this.P; }
    public double getI() { return this.I; }
    public double getD() { return this.D; }
    public double getSetpoint() { return this.setpoint; }
    public FFbase getFF(){ return this.FF; }

    public static class PIDBuilder {
        double
            P, 
            I, 
            D,
            setpoint,
            maxInt = 0,
            minInt = 0,
            izone = 0;
        FFbase kFF = new FFbase();
        String name = "";
        public PIDBuilder (double kSetpoint, double kP, double kI, double kD){
            this.setpoint = kSetpoint;
            this.P = kP;
            this.I = kI;
            this.D = kD;
        }
        public PID build(){
            return new PID(this);
        }
        public PIDBuilder FF (FFbase kFF){
            this.kFF = kFF;
            return this;
        }
        public PIDBuilder MinIntegral(double minint){
            this.minInt = minint;
            return this;
        }
        public PIDBuilder MaxIntegral(double maxint){
            this.maxInt = maxint;
            return this;
        }
        public PIDBuilder IZone(double kizone){
            this.izone = kizone;
            return this;
        }
        public PIDBuilder Name(String name){
            this.name = name;
            return this;
        }
    }
}
