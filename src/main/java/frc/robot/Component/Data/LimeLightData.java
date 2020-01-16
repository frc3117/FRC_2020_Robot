package frc.robot.Component.Data;

public class LimeLightData {
    public LimeLightData(double tx, double ty, double ta, double tv)
    {
        _tx = tx;
        _ty = ty;
        _ta = ta;
        _tv = tv;
    }

    private double _tx;
    private double _ty;
    private double _ta;
    private double _tv;

    public double GetAngleX()
    {
        return _tx;
    }
    public double GetAngleY()
    {
        return _ty;
    }
    public double GetScreenSpace()
    {
        return _ta;
    }
    public boolean IsTarget()
    {
        return _tv == 1;
    }
}
