package frc.robot.Component;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.MotorController.MotorControllerType;

public class BallIntake 
{
    public BallIntake(int ControlerChannel, int leftSolenoidChannel, int rightSolenoidChannel)
    {
        _controller = new MotorController(MotorControllerType.TalonSRX, ControlerChannel, false);

        _leftSolenoid = new Solenoid(leftSolenoidChannel);
        _rightSolenoid = new Solenoid(rightSolenoidChannel);
    }

    private boolean _isStarted = false;

    public MotorController _controller;

    private Solenoid _leftSolenoid;
    private Solenoid _rightSolenoid;

    public void Init()
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
            _leftSolenoid.set(true);
            _rightSolenoid.set(true);

            _controller.Set(0.6);
        }
        else
        {
            _leftSolenoid.set(false);
            _rightSolenoid.set(false);

            _controller.Set(0);
        }
    }
}
