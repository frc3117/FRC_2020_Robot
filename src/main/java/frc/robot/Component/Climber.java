package frc.robot.Component;

import frc.robot.Robot;
import frc.robot.Component.Data.InputManager;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Interface.System;

public class Climber implements System
{
    public Climber(int MotorChannel)
    {
        _climberMotor = new MotorController(MotorControllerType.SparkMax, MotorChannel, true);
    }

    private MotorController  _climberMotor;

    public void Init()
    {
    }

    public void DoSystem()
    {
        if(InputManager.GetButton("ClimberUp"))
        {
            _climberMotor.Set(-0.75);
            Robot.Intake.OpenIntake();
        }
        else if (InputManager.GetButton("ClimberDown"))
        {
            _climberMotor.Set(0.35);
            Robot.Intake.OpenIntake();
        }
        else
        {
            _climberMotor.Set(0);
        }
    }
}
