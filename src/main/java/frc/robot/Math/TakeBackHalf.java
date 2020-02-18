package frc.robot.Math;

/**
 * The take back half controller class
 */
public class TakeBackHalf 
{
    public TakeBackHalf(double Gain)
    {
        _gain = Gain;
    }

    private double _gain;

    private double _lastError = 0;
    private double _output = 0;
    private double _tbh = 0;

    /**
     * Evaluate the current take back half controller
     * @param Error The error of the controller
     * @return The current take back half controller
     */
    public double Evaluate(double Error)
    {
        _output += _gain * Error;
        if (Math.signum(Error) != Math.signum(_lastError)) 
        {
            _output = 0.5 * (_output + _tbh);
            _tbh = _output;
            _lastError = Error;
        }

        return _output;
    }

    /**
     * Reset the take back half controller
     */
    public void Reset()
    {
        _lastError = 0;
        _output = 0;
        _tbh = 0;
    }
}
