package frc.robot.Component;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

public class BallIntake 
{
    public BallIntake(double IdleSpeed,int ControlerChannel, int channelA, int channelB)
    {
        _idleSpeed = IdleSpeed;

        _controller = new MotorController(MotorControllerType.TalonSRX, ControlerChannel, false);
        //_encoder = new Encoder(channelA, channelB);
    }

    private double _idleSpeed;
    private boolean _isStarted = false;

    public MotorController _controller;
    private Encoder _encoder;
    private PID _inertialWheelPID = new PID(0, 0, 0);

    public void InitIntake()
    {
        _isStarted = false;
    }

    public void StartIntake()
    {
        _isStarted = true;
    }
    public void StopIntake()
    {
        _isStarted = false;
    }

    public void DoIntake()
    {
        if(_isStarted)
        {

        }
        else
        {
            _controller.Set(Math.max(0, _inertialWheelPID.Evaluate(_idleSpeed - _encoder.getRate(), Timer.GetDeltaTime())));
        }
    }
}
