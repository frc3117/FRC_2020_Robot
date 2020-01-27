package frc.robot.Component.Data;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Math.PID;

public class MotorController 
{
    public MotorController(MotorControllerType type, int Channel, boolean IsBrushless)
    {
        _controllerType = type;

        switch(type)
        {
            case SparkMax:
            _sparkMax = new CANSparkMax(Channel, IsBrushless ? MotorType.kBrushless : MotorType.kBrushed);
            break;

            case TalonSRX:
            _talonSRX = new WPI_TalonSRX(Channel);
            break;
        }

        _pid = new PID(0, 0, 0);
        _usePID = false;
    }
    public MotorController(MotorControllerType type, int Channel, boolean IsBrushless, int EncoderChannelA, int EncoderChannelB, double Kp, double Ki, double Kd)
    {
        _controllerType = type;

        switch(type)
        {
            case SparkMax:
            _sparkMax = new CANSparkMax(Channel, IsBrushless ? MotorType.kBrushless : MotorType.kBrushed);
            break;

            case TalonSRX:
            _talonSRX = new WPI_TalonSRX(Channel);
            break;
        }

        _encoder = new Encoder(EncoderChannelA, EncoderChannelB);

        _pid = new PID(Kp, Ki, Kd);
        _usePID = true;
    }

    public enum MotorControllerType
    {
        SparkMax,
        TalonSRX
    }

    private MotorControllerType _controllerType;
    private PID _pid;

    private boolean _usePID;

    private Encoder _encoder;
    private int _encoderResolution = 2048;

    private CANSparkMax _sparkMax;
    private WPI_TalonSRX _talonSRX;

    public void SetPID(double Kp, double Ki, double Kd)
    {
        _pid.SetGain(Kp, Ki, Kd);
    }
    public void UsePID(boolean state)
    {
        _usePID = state;
    }
    public void SetDebugPID(String Name)
    {
        _pid.SetDebugMode(Name);
    }
    public void StopDebugPID()
    {
        _pid.StopDebugMode();
    }

    public void SetEncoderResolution(int Resolution)
    {
        _encoderResolution = Resolution;
    }

    public void Set(double Value)
    {
        switch(_controllerType)
        {
            case SparkMax:
            if(_usePID)
            {
                _sparkMax.set(_pid.Evaluate(Value - (_encoder.getRate() / _encoderResolution)));
            }
            else
            {
                _sparkMax.set(Value);
            }
            break;

            case TalonSRX:
            if(_usePID)
            {
                _talonSRX.set(_pid.Evaluate(Value - (_encoder.getRate() / _encoderResolution)));
            }
            else
            {
                _talonSRX.set(Value);
            }
            break;
        }
    }
    public double GetEncoderVelocity()
    {
        return _encoder.getRate() / _encoderResolution;
    }
}
