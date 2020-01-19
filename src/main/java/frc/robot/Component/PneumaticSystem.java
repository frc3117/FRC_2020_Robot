/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component;

import edu.wpi.first.wpilibj.Compressor;

public class PneumaticSystem 
{
    private static Compressor _compressor;
    private static boolean _isRuning;

    public static void Init()
    {
        _compressor = new Compressor();
    }

    public static void CheckPressure()
    {
        if(_isRuning && _compressor.getPressureSwitchValue())
        {
            _compressor.stop();
            _isRuning = false;
        }
        else if (!_isRuning && !_compressor.getPressureSwitchValue())
        {
            _compressor.start();
            _isRuning = true;
        }
    }
}
