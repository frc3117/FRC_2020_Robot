package frc.robot.Component;

import frc.robot.Interface.System;
import frc.robot.Math.Timer;

public class Climber implements System
{
    public Climber()
    {

    }

    public void Init()
    {

    }

    public void DoSystem()
    {
        //Allow the climber to move only in the last 30 seconds
        if(Timer.GetPeriodTime() <= 30)
        {
            
        }
    }
}
