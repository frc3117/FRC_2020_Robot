/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component.Data;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class AutonomousSequenceAction 
{
    private AutonomousSequenceAction(double Time, ActionType Type)
    {
        _time = Time;
        _type = Type;
    }
    private AutonomousSequenceAction(double Time, double Number, ActionType Type)
    {
        _time = Time;
        _num = Number;
        _type = Type;
    }
    private AutonomousSequenceAction(double Time, Vector2d Vector, ActionType Type)
    {
        _time = Time;
        _vec = Vector;
        _type = Type;
    }

    public enum ActionType
    {
        Wait,
        RotateToward,
        Rotate,
        Translate,
        MoveTo
    }
    
    private ActionType _type;

    private double _time;

    private Vector2d _vec;
    private double _num;

    public static AutonomousSequenceAction CreateWait(double Time)
    {
        return new AutonomousSequenceAction(Time, ActionType.Wait);
    }
    public static AutonomousSequenceAction CreateRotateToward(double Time, double TargetAngle)
    {
        return new AutonomousSequenceAction(Time, TargetAngle, ActionType.RotateToward);
    }
    public static AutonomousSequenceAction CreateRotate(double Time, double Angle)
    {
        return new AutonomousSequenceAction(Time, Angle, ActionType.Rotate);
    }
    public static AutonomousSequenceAction CreateTranslate(double Time, Vector2d Translation)
    {
        return new AutonomousSequenceAction(Time, Translation, ActionType.Translate);
    }
    public static AutonomousSequenceAction CreateMoveTo(double Time, Vector2d TargetPosition)
    {
        return new AutonomousSequenceAction(Time, TargetPosition, ActionType.MoveTo);
    }

    public ActionType GetType()
    {
        return _type;
    }

    public double GetTime()
    {
        return _time;
    }
    public Vector2d GetVector()
    {
        return _vec;
    }
    public double GetNumber()
    {
        return _num;
    }
}
