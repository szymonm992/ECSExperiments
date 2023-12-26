using Unity.Entities;
using Unity.Mathematics;

public struct WheelHitData : IComponentData
{
    public float3 Origin;
    public float3 WheelCenter;
    public float3 Position;
    public float SurfaceFriction;
    public float3 Velocity;
    public bool HasHit;

    public void Reset()
    {
        Origin = float3.zero;
        WheelCenter = float3.zero; 
        Position = float3.zero;
        SurfaceFriction = 0;
        HasHit = false;
        Velocity = float3.zero;
    }
}
