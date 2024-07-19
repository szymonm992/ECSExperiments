using Unity.Entities;
using Unity.Mathematics;

public struct WheelHitData : IComponentData
{
    public float3 WheelCenter;
    public float3 HitPoint;
    public float3 VelocityAtContactPoint;
    public float3 Normal;

    public float SurfaceFriction;
    public float Distance;
 
    public bool HasHit;
}
