/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Math;

public class PID {
    
    public PID() {}
    public PID(double KP, double KI, double KD)
    {
        SetGain(KP, KI, KD);
    }

    public double Kp = 0;
    public double Ki = 0;
    public double Kd = 0;

    public double Tolerancy = 0;

    private double previousError = 0;
    private double previousAverageError = 0;
    private double integral = 0;

    public void SetGain(double KP, double KI, double KD)
    {
        Kp = KP;
        Ki = KI;
        Kd = KD;
    }
    public void SetTolerancy(double tolerancy)
    {
        Tolerancy = tolerancy;
    }

    public double Evaluate(double Error, double Dt)
    {
        if(Tolerancy > Math.abs(Error))
        {
            Error = 0;
        }

        integral += Error * Dt;

        double averageError = (Error + previousError) / 2;
        double derivative = (averageError - previousAverageError) / Dt;

        previousError = Error;
        previousAverageError = averageError;

        return Kp * Error + Ki * integral + Kd * derivative;
    }

    public void Reset()
    {
        previousError = 0;
        integral = 0;
    }
}
