package frc.robot.Component;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Robot;
import frc.robot.Component.Data.InputManager;
import frc.robot.Interface.System;
import frc.robot.Math.Timer;

public class Climber implements System
{
    public Climber(int ShifterChannel)
    {
        _ballShifter = new Solenoid(ShifterChannel);
    }

    private boolean _shifterState = false;
    private Solenoid _ballShifter;

    public void Init()
    {
        _shifterState = false;
        _ballShifter.set(false);
    }

    public void DoSystem()
    {
        //Allow the climber to move only in the last 30 seconds
        if(Timer.GetPeriodTime() <= 30)
        {
            if(InputManager.GetButtonDown("ToggleClimber"))
            {
                _shifterState = !_shifterState;
                _ballShifter.set(_shifterState);

                Robot.Thrower.SetClimbMode(_shifterState);
            }
        }
    }
}
