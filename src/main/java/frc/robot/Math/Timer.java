package frc.robot.Math;

/**
 * Add your docs here.
 */
public class Timer 
{
    private static double _startTime = 0;

    private static double _lastTime = 0;
    private static double _dt = 0;

    public static void Init()
    {
        _lastTime = System.nanoTime();
        _startTime = GetCurrentTime();
    }
    public static double Calculate()
    {
        //Estimate the delta time betwen the last time
        long currentTime = System.nanoTime();
        _dt = (currentTime - _lastTime) / 1e9;
        _lastTime = currentTime; 

        return currentTime;
    }
    public static double GetDeltaTime()
    {
        //The last estimated delta time
        return _dt;
    }
    public static double GetCurrentTime()
    {
        //The current time of the RoboRio
        return System.nanoTime() / 1e9;
    }
    public static double GetTimeSinceStart()
    {
        return GetCurrentTime() - _startTime;
    }
}
