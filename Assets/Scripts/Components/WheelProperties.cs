using Unity.Entities;

public struct WheelProperties : IComponentData
{
    public float Spring;
    public float Damper;
    public float Mass;
    public float Radius;
    public float Thickness;
    public float Travel;
}
