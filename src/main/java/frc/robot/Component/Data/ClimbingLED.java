package frc.robot.Component.Data;

import frc.robot.Robot;
import frc.robot.Interface.Component;

/**
 * Add your docs here.
 */
public class ClimbingLED implements Component
{
    public ClimbingLED(int blinkCount, double blinkTime)
    {
        _count = blinkCount;
        _time = blinkTime;
    }

    private int _count;
    private double _time;

    private boolean _isAnimation;
    private double _startTime;

    public void Init()
    {
        _isAnimation = false;
        _startTime = 0;
    }

    public void StartAnimation()
    {
        System.out.println("Start Anim");

        _isAnimation = true;
        _startTime = frc.robot.Math.Timer.GetCurrentTime();
    }
    public void StopAnimation()
    {
        _isAnimation = false;
        _startTime = 0;

        Robot.Leds.SetColor("off", 999, 0);
    }

    public void DoSystem()
    {
        if((frc.robot.Math.Timer.GetCurrentTime() - _startTime) >= (_count) * _time)
        {
            _isAnimation = false;
            Robot.Leds.SetColor("random", 101, 100);
        }

        if(!_isAnimation)
            return;

        double currentCycle = (frc.robot.Math.Timer.GetCurrentTime() - _startTime) % _time;

        if(currentCycle >= _time / 5.)
        {
            Robot.Leds.SetColor("white", 101, 100);
        }
        else
        {
            Robot.Leds.SetColor("lightpurple", 101, 100);
        }
    }
}
