package frc.robot.Math;

/**
 * The rate limiter class
 */
public class RateLimiter 
{
    public RateLimiter(double Velocity, double InitValue)
    {
        _current = InitValue;
    }

    private double _velocity;
    private double _current;

    /**
     * Evaluate the current value of the rate limiter
     * @param TargetValueThe target value of the rate limiter
     * @return The evaluate current value of the rate limiter
     */
    public double Evaluate(double TargetValue)
    {
        return Evaluate(TargetValue, Timer.GetDeltaTime());
    }
    /**
     * Evaluate the current value of the rate limiter
     * @param TargetValue The target value of the rate limiter
     * @param Dt The delta time since the last evaluation
     * @return The evaluate current value of the rate limiter
     */
    public double Evaluate(double TargetValue, double Dt)
    {
        if(Math.abs(TargetValue - _current) <= _velocity * Dt)
        {
            _current = TargetValue;
        }
        else
        {
            _current += (Math.signum(TargetValue - _current)) * _velocity * Dt;
        }

        return _current;
    }

    /**
     * Get the current value of the rate limiter
     * @return The current value of the rate limiter
     */
    public double GetCurrent()
    {
        return _current;
    }
}
