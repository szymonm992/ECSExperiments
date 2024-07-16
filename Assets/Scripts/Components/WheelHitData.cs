using Unity.Entities;
using Unity.Mathematics;

public struct WheelHitData : IComponentData
{
    public float3 WheelCenter;
    public float3 HitPoint;
    public float3 Velocity;
    public float3 Normal;

    public float SurfaceFriction;
    public float Distance;
 
    public bool HasHit;

    public void Reset()
    {
        WheelCenter = float3.zero; 
        HitPoint = float3.zero;
        Velocity = float3.zero;
        Normal = float3.zero;

        SurfaceFriction = 0;
        Distance = 0;

        HasHit = false;
    }
}
