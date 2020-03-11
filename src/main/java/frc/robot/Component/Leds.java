package frc.robot.Component;

import java.util.HashMap;
import java.util.Random;

import frc.robot.Component.Data.SolenoidValve;
import frc.robot.Interface.Component;
import frc.robot.Math.Timer;

/**
 * The LED controller
 */
public class Leds implements Component {
    private SolenoidValve green;
    private SolenoidValve blue;
    private SolenoidValve red;

    private String _color = "";
    private Integer _priority = 0;

    private HashMap<String, ColorCycle> _cycle;
    private double _startTime;
    private int _currentCycleIndex = 0;

    public Leds(int greenChannel, int blueChannel, int redChannel) {
        green = SolenoidValve.CreateSingle(greenChannel, 1);
        blue = SolenoidValve.CreateSingle(blueChannel, 1);
        red = SolenoidValve.CreateSingle(redChannel, 1);
    
        _cycle = new HashMap<String, ColorCycle>();
    }

    public void Init() {
        _color = "off";
        _priority = 0;
    }

    /**
     * Set the new color of the led strip
     * @param color The new color
     * @param priority The priority of the new color
     * @param newPriority The priority to keep in memory
     */
    public void SetColor(String color, Integer priority, Integer newPriority) {
        if (priority >= _priority) {
            _priority = newPriority;
            _color = color;

            if(_cycle.containsKey(color))
            {
                _startTime = Timer.GetCurrentTime();
                _currentCycleIndex = 0;
            }
        }
    }

    /**
     * Add a new color cycle
     * @param CycleName The name of the color cycle
     * @param Cycle The new color cycle
     */
    public void AddColorCycle(String CycleName, String Cycle)
    {
        if(_cycle.containsKey(CycleName))
            return;

        String[] Split = Cycle.split(":", 0);

        String[] color = new String[Split.length];
        double[] time = new double[Split.length];

        for(int i = 0; i < Split.length; i++)
        {
            String[] data = Split[i].split("_", 0);

            color[i] = data[0];
            time[i] = Double.parseDouble(data[1]);
        }

        ColorCycle current = new ColorCycle();
        current.Color = color;
        current.Time = time;

        _cycle.put(CycleName, current);
    }

    public void DoSystem() {  
        String CurrentColor;

        if(_cycle.containsKey(_color))
        {
            ColorCycle current = _cycle.get(_color);

            if(current.Time[_currentCycleIndex] <= Timer.GetCurrentTime() - _startTime)
            {
                if(++_currentCycleIndex == current.Time.length)
                {
                    _currentCycleIndex = 0;
                }

                _startTime = Timer.GetCurrentTime();
            }

            CurrentColor = current.Color[_currentCycleIndex];
        }
        else
        {
            CurrentColor = _color;
        }
        
        switch (CurrentColor) {
            case "green":
                green.SetState(true);
                blue.SetState(false);
                red.SetState(false);
            break;

            case "blue":
                green.SetState(false);
                blue.SetState(true);
                red.SetState(false);
            break;

            case "red":
                green.SetState(false);
                blue.SetState(false);
                red.SetState(true);
            break;

            case "lightblue":
                green.SetState(true);
                blue.SetState(true);
                red.SetState(false);
            break;

            case "lime":
                green.SetState(true);
                blue.SetState(false);
                red.SetState(true);
            break;

            case "lightpurple":
                green.SetState(false);
                blue.SetState(true);
                red.SetState(true);
            break;

            case "white":
                green.SetState(true);
                blue.SetState(true);
                red.SetState(true);
            break;

            case "random":
                green.SetState(new Random().nextInt(2) == 1);
                blue.SetState(new Random().nextInt(2) == 1);
                green.SetState(new Random().nextInt(2) == 1);
            break;

            case "off":
            default:
                green.SetState(false);
                blue.SetState(false);
                red.SetState(false);
            break;
        }
    }

    private class ColorCycle
    {
        public String[] Color;
        public double[] Time;
    }
}