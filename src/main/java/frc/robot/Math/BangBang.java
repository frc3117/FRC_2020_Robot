package frc.robot.Math;

/**
 * The bang bang controller class
 */
public class BangBang 
{
    public BangBang(double PositiveResult, double NeurtralResult, double NegativeResult)
    {
        _positive = PositiveResult;
        _neutral = NeurtralResult;
        _negative = NegativeResult;
    }

    private double _positive;
    private double _neutral;
    private double _negative;

    private double _tolerency = 0;

    private double _target = 0;

    /**
     * Set the tolerency of the bang bang controller (Range in wich controller will be at neutral)
     * @param Tolerency The tolerency of the bang bang controller
     */
    public void SetTolerency(double Tolerency)
    {
        _tolerency = Tolerency;
    }

    /**
     * Set the target of the bang bang controller
     * @param Target The target of the bang bang controller
     */
    public void SetTarget(double Target)
    {
        _target = Target;
    }

    /**
     * Get the current value of the bang bang controller
     * @param Current The current value to evaluate
     * @return The current value of the bang bang controller
     */
    public double Evaluate(double Current)
    {
        double error = _target - Current;

        if(Math.abs(error) <= _tolerency)
        {
            return _neutral;
        }

        return Math.signum(error) == 1 ? _positive : _negative;
    }
}
