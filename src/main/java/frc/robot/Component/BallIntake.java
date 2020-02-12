package frc.robot.Component;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Component.Data.Input;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Math.PID;

public class BallIntake 
{
    public BallIntake(int ControlerChannel, int SolenoidChannelA, int SolenoidChannelB, int EncoderA, int EncoderB, int TargetRPM)
    {
        _controller = new MotorController(MotorControllerType.TalonSRX, ControlerChannel, false);

        _motorEncoder = new Encoder(EncoderA, EncoderB);

        _solenoid = new DoubleSolenoid(SolenoidChannelA, SolenoidChannelB);

        _targetSpeed = TargetRPM;
        Input.CreateButton("ToggleIntake", 0, 4);

    }

    private double _targetSpeed;
    private boolean _isStarted = false;

    private Encoder _motorEncoder;
    private PID _motorPID = new PID(0.001, 0, 0);
    private MotorController _controller;

    private DoubleSolenoid _solenoid;

    private boolean _lastInput = false;

    public void Init()
    {
        _isStarted = false;
    }

    public void StartIntake()
    {
        _isStarted = true;
        _solenoid.set(Value.kForward);
    }
    public void StopIntake()
    {
        _isStarted = false;
        _solenoid.set(Value.kReverse);
    }
    public void ToggleIntake()
    {
        _isStarted = !_isStarted;

        if(_isStarted)
        {
            _solenoid.set(Value.kForward);
        }
        else
        {
            _solenoid.set(Value.kReverse);
        }
    }

    public void DoIntake()
    {
        boolean current = Input.GetButton(("ToggleIntake"));
        if(current && !_lastInput)
        {
          ToggleIntake();
        }
        _lastInput = current;

        if(_isStarted)
        {        
            
            double currentSpeed = (_motorEncoder.getRate() / 2048) * 60;
            _controller.Set(_motorPID.Evaluate(_targetSpeed - currentSpeed));
        }
        else
        {
        }    
    }
}
