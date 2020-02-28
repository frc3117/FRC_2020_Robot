package frc.robot.Component;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Component.Data.Input;
import frc.robot.Component.Data.InputManager;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Interface.System;
import frc.robot.Math.Mathf;
import frc.robot.Math.PID;

public class BallIntake implements System
{
    public BallIntake(int ControlerChannel, int SolenoidChannelA, int SolenoidChannelB, int EncoderA, int EncoderB, int TargetRPM)
    {
        _controller = new MotorController(MotorControllerType.TalonSRX, ControlerChannel, false);

        _motorEncoder = new Encoder(EncoderA, EncoderB);

        _solenoid = new DoubleSolenoid(SolenoidChannelA, SolenoidChannelB);

        _targetSpeed = TargetRPM;
    }

    private double _targetSpeed;
    private boolean _isOpen = false;

    private Encoder _motorEncoder;
    private PID _motorPID = new PID(0.002, 0.00001, 0, "Feeder");
    private MotorController _controller;

    private DoubleSolenoid _solenoid;

    public void Init()
    {
        _isOpen = false;
    }

    public void OpenIntake()
    {
        _isOpen = true;
        _solenoid.set(Value.kForward);
    }
    public void CloseIntake()
    {
        _isOpen = false;
        _solenoid.set(Value.kReverse);
    }
    public void ToggleIntake()
    {
        _isOpen = !_isOpen;

        if(_isOpen)
        {
            _solenoid.set(Value.kForward);
        }
        else
        {
            _solenoid.set(Value.kReverse);
        }
    }
    public boolean IsOpen()
    {
        return _isOpen;
    }

    public void DoSystem()
    {
        if(InputManager.GetButtonDown("ToggleIntake"))
        {
            ToggleIntake();
        }

        if(_isOpen)
        {        
            double currentSpeed = (_motorEncoder.getRate() / 2048) * 60;

            if(Input.GetButton("StartFeeder"))
            {
                if(Input.GetButton("ReverseFeeder"))
                    _controller.Set(1);
                else
                    _controller.Set(-1);          
            }
            else
            {
                _controller.Set(0);
            }

            SmartDashboard.putNumber("FeederSpeed", currentSpeed);
        }
        else
        {
            _controller.Set(0);
        }    
    }
}