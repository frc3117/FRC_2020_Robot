/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component.Data;

public class TangentCurveKey 
{
    public TangentCurveKey(double x, double y, double inTangent, double outTangent)
    {
        X = x;
        Y = y;

        InTangent = inTangent;
        OutTangent = outTangent;
    }

    public double X;
    public double Y;
    
    public double InTangent;
    public double OutTangent;
}
