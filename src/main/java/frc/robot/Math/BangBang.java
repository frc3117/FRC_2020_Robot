package frc.robot.Math;


public class BangBang 
{
    public BangBang(double PositiveResult, double NegativeResult)
    {
        _positive = PositiveResult;
        _negative = NegativeResult;
    }

    private double _positive;
    private double _negative;

    private double _target = 0;

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

        return Math.signum(error) == 1 ? _positive : _negative;
    }
}
