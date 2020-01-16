/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Math;

/**
 * Add your docs here.
 */
public class Timer 
{
    private static double _lastTime = 0;
    private static double _dt = 0;

    public static void Init()
    {
        _lastTime = System.nanoTime();
    }
    public static double Calculate()
    {
        long currentTime = System.nanoTime();
        _dt = (currentTime - _lastTime) / 1e9;
        _lastTime = currentTime; 

        return currentTime;
    }
    public static double GetDeltaTime()
    {
        return _dt;
    }
    public static double GetCurrentTime()
    {
        return System.nanoTime() / 1e9;
    }
}
