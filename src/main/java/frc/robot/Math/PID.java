package frc.robot.Math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID {
    
    public PID() {}
    public PID(double KP, double KI, double KD)
    {
        SetGain(KP, KI, KD);
    }
    public PID(double KP, double KI, double KD, String DebugName)
    {
        SetGain(KP, KI, KD);
        SetDebugMode(DebugName);
    }

    public double Kp = 0;
    public double Ki = 0;
    public double Kd = 0;

    public double Tolerancy = 0;

    private double _feedFoward;

    private double _previousError = 0;
    private double _previousAverageError = 0;
    private double _integral = 0;

    private boolean _isDebug = false;
    private String _kpName;
    private String _kiName;
    private String _kdName;

    public void SetDebugMode(String Name)
    {
        if(!_isDebug)
        {
            Reset();

            _isDebug = true;

            _kpName = Name + "_Kp";
            _kiName = Name + "_Ki";
            _kdName = Name + "_Kd";

            SmartDashboard.putNumber(_kpName, Kp);
            SmartDashboard.putNumber(_kiName, Ki);
            SmartDashboard.putNumber(_kdName, Kd);
        }
    }
    public void StopDebugMode()
    {
        if(_isDebug)
        {
            Reset();

            _isDebug = false;
            
            SmartDashboard.delete(_kpName);
            SmartDashboard.delete(_kiName);
            SmartDashboard.delete(_kdName);
        }
    }

    public void SetGain(double KP, double KI, double KD)
    {
        Kp = KP;
        Ki = KI;
        Kd = KD;
    }
    public void SetTolerancy(double tolerancy)
    {
        Tolerancy = tolerancy;
    }

    public void SetFeedForward(double FeedForward)
    {
        _feedFoward = FeedForward;
    }

    public double Evaluate(double Error)
    {
        return Evaluate(Error, Timer.GetDeltaTime());
    }
    public double Evaluate(double Error, double Dt)
    {
        if(Tolerancy > Math.abs(Error))
        {
            Error = 0;
        }

        _integral += Error * Dt;

        double averageError = (Error + _previousError) / 2;
        double derivative = (averageError - _previousAverageError) / Dt;

        _previousError = Error;
        _previousAverageError = averageError;

        if(_isDebug)
        {
            return SmartDashboard.getNumber(_kpName, 0) * Error + SmartDashboard.getNumber(_kiName, 0) * _integral + SmartDashboard.getNumber(_kdName, 0) * derivative + _feedFoward;
        }
        else
        {
            return Kp * Error + Ki * _integral + Kd * derivative + _feedFoward;
        }
    }

    public void Reset()
    {
        _previousError = 0;
        _integral = 0;
    }
}
