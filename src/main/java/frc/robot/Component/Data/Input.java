/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component.Data;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;

public class Input 
{
    private Input(int ID,  int input,  String inputName, boolean invert) {
        if (!_joysticks.containsKey(ID)) {
            _joysticks.put(ID, new Joystick(ID));
        }

        _joystickID = ID;
        _input = input;
        _isInputNegativeInverted = invert;

        _inputs.put(inputName, this);
    }

    private static HashMap<String, Input> _inputs = new HashMap<String, Input>();
    private static HashMap<Integer, Joystick> _joysticks = new HashMap<Integer, Joystick>();

    private int _joystickID;
    private int _joystickIDNegative = 9999;
    private int _input; // The ID of the input
    private boolean _isInputInverted;
    private int _inputNegative = 9999;
    private boolean _isInputNegativeInverted;

    public static void CreateAxis(String Name, int JoystickID, int InputID, boolean invert) 
    {
        new Input(JoystickID, InputID, "Axis/" + Name, invert);
    }
    public static void CreateButton(String Name, int JoystickID, int InputID) 
    {
        new Input(JoystickID, InputID, "Button/" + Name, false);
    }

    public static boolean ContainAxis(String Name)
    {
        return _inputs.containsKey("Axis/" + Name);
    }
    public static boolean ContainButton(String Name)
    {
        return _inputs.containsKey("Button/" + Name);
    }

    public static void SetAxisNegative(String Name, int JoystickID, int InputID, boolean invert)
    {
        if(!_joysticks.containsKey(JoystickID))
        {
            _joysticks.put(JoystickID, new Joystick(JoystickID));
        }

        Input current = _inputs.get("Axis/" + Name);

        current._joystickIDNegative = JoystickID;
        current._inputNegative = InputID;

        current._isInputNegativeInverted = invert;
    }

    public static void Reset() {
        _inputs.clear();
        _joysticks.clear();
    }

    public static double GetAxis(String Name) {
        Input current = _inputs.get("Axis/" + Name);
        double negative = 0;

        if(current._joystickIDNegative != 9999)
        {
            negative = _joysticks.get(current._joystickIDNegative).getRawAxis(current._inputNegative) * (current._isInputNegativeInverted ? -1 : 1);
        }

        return (_joysticks.get(current._joystickID).getRawAxis(current._input) * (current._isInputInverted ? -1 : 1)) - negative;
    }

    public static boolean GetButton(String Name) {
         Input current = _inputs.get("Button/" + Name);

        return _joysticks.get(current._joystickID).getRawButton(current._input);
    }
}