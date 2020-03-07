package frc.robot.Interface;

import java.util.HashMap;

/**
 * The base interface of the robot
 */
public interface RobotBase
{
    public HashMap<String, Component> Systems = new HashMap<String, Component>();

    public void robotInit();

    public void disabledInit();
    public void disabledPeriodic();

    public void autonomousInit();
    public void autonomousPeriodic();

    public void teleopInit();
    public void teleopPeriodic();

    public void Init();
}
