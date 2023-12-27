using Unity.Entities;

public struct WheelProperties : IComponentData
{
    public Entity Entity;
    public float Spring;
    public float Damper;
    public float Mass;
    public float Radius;
    public float Thickness;
    public float SpringLength;
    public bool IsGrounded;
    public bool CanDrive;
    public WheelSide Side;
}
