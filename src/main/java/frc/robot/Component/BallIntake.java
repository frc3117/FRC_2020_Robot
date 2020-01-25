package frc.robot.Component;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Math.Mathf;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

public class BallIntake 
{
    public BallIntake(double IdleSpeed, double ShootSpeed, int ControlerChannel, int channelA, int channelB, int leftSolenoidChannel, int rightSolenoidChannel)
    {
        _idleSpeed = IdleSpeed;
        _shootSpeed = ShootSpeed;
        
        _controller = new MotorController(MotorControllerType.TalonSRX, ControlerChannel, false);
        //_encoder = new Encoder(channelA, channelB);

        _leftSolenoid = new Solenoid(leftSolenoidChannel);
        _rightSolenoid = new Solenoid(rightSolenoidChannel);
    }

    private double _idleSpeed;
    private double _shootSpeed;
    private boolean _isStarted = false;

    public MotorController _controller;
    private Encoder _encoder;
    private PID _inertialWheelPID = new PID(0, 0, 0);

    private Solenoid _leftSolenoid;
    private Solenoid _rightSolenoid;

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
            _controller.Set(Mathf.Clamp(_inertialWheelPID.Evaluate(_shootSpeed - (_encoder.getRate() / 34.1333333333), Timer.GetDeltaTime()), 0, 1));

            _leftSolenoid.set(true);
            _rightSolenoid.set(true);
        }
        else
        {
            _controller.Set(Math.max(0, _inertialWheelPID.Evaluate(_idleSpeed - (_encoder.getRate() / 34.1333333333), Timer.GetDeltaTime())));

            _leftSolenoid.set(false);
            _rightSolenoid.set(false);
        }
    }
}
