package frc.robot.Component;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Component.Data.Input;
import frc.robot.Component.Data.InputManager;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.SolenoidValve;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Interface.Component;

public class BallIntake implements Component
{
    public BallIntake(int ControlerChannel, int SolenoidChannelA, int SolenoidChannelB, int EncoderA, int EncoderB)
    {
        _controller = new MotorController(MotorControllerType.TalonSRX, ControlerChannel, false);
        _motorEncoder = new Encoder(EncoderA, EncoderB);
        _solenoid = SolenoidValve.CreateDouble(SolenoidChannelA, SolenoidChannelB, 0);
    }

    private boolean _isOpen = false;

    private Encoder _motorEncoder;
    private MotorController _controller;

    private SolenoidValve _solenoid;

    private boolean _isIntakeOverriden;
    private double _intakeOverrideValue;

    public void Init()
    {
        _isOpen = false;
    }

    public void OpenIntake()
    {
        _isOpen = true;
        _solenoid.SetState(true);
    }
    public void CloseIntake()
    {
        _isOpen = false;
        _solenoid.SetState(false);
    }
    public void ToggleIntake()
    {
        _isOpen = !_isOpen;

        _solenoid.SetState(_isOpen);
    }
    public boolean IsOpen()
    {
        return _isOpen;
    }

    public void OverrideIntake(double Speed)
    {
        _isIntakeOverriden = true;
        _intakeOverrideValue = Speed;
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

            if(_isIntakeOverriden)
            {
                _controller.Set(_intakeOverrideValue);
            }
            else
            {
                if(Input.GetButton("StartFeeder"))
                {
                    _controller.Set(-0.8);          
                }
                else if(Input.GetButton("ReverseFeeder"))
                {
                    _controller.Set(0.8);
                }
                else
                {
                    _controller.Set(0);
                }
            }

            SmartDashboard.putNumber("FeederSpeed", currentSpeed);
        }
        else
        {
            _controller.Set(0);
        }    

        _isIntakeOverriden = false;
    }
}