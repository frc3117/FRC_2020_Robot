package frc.robot.Component.Data;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Math.PID;

/**
 * The universal motor controller
 */
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

            case TalonFX:
            _talonFX = new TalonFX(Channel);
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

            case TalonFX:
            _talonFX = new TalonFX(Channel);
            break;
        }

        _encoder = new Encoder(EncoderChannelA, EncoderChannelB);

        _pid = new PID(Kp, Ki, Kd);
        _usePID = true;
    }

    public enum MotorControllerType
    {
        SparkMax,
        TalonSRX,
        TalonFX
    }

    private MotorControllerType _controllerType;
    private PID _pid;

    private boolean _usePID;

    private Encoder _encoder;
    private int _encoderResolution = 2048;

    private CANSparkMax _sparkMax;
    private WPI_TalonSRX _talonSRX;
    private TalonFX _talonFX;

    /**
     * Set the pid gain of the motor controller
     * @param Kp The proportional gain
     * @param Ki The integral gain
     * @param Kd The derivative gain
     */
    public void SetPID(double Kp, double Ki, double Kd)
    {
        _pid.SetGain(Kp, Ki, Kd);
    }
    /**
     * If the pid is used
     * @param state The new state of the pid usage
     */
    public void UsePID(boolean state)
    {
        _usePID = state;
    }
    /**
     * Set the pid in debug mode
     * @param Name The name to show it as in the SmartDashboard
     */
    public void SetDebugPID(String Name)
    {
        _pid.SetDebugMode(Name);
    }
    /**
     * Stop the pid debug mode
     */
    public void StopDebugPID()
    {
        _pid.StopDebugMode();
    }

    /**
     * Set the resolution of the encoder
     * @param Resolution The resolution of the encoder
     */
    public void SetEncoderResolution(int Resolution)
    {
        _encoderResolution = Resolution;
    }

    /**
     * Set the value to send to the motor contoller
     * @param Value The value to send to the motor controller
     */
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

            case TalonFX:
            if(_usePID)
            {
                _talonFX.set(TalonFXControlMode.PercentOutput, _pid.Evaluate(Value - (_encoder.getRate() / _encoderResolution)));
            }
            else
            {
                _talonFX.set(TalonFXControlMode.PercentOutput, Value);
            }
            break;
        }
    }
    /**
     * Get the instantaneous velocity of the encoder
     * @return The instantaneous velocity of the encoder
     */
    public double GetEncoderVelocity()
    {
        return _encoder.getRate() / _encoderResolution;
    }
}
