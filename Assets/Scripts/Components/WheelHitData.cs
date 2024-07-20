using Unity.Entities;
using Unity.Mathematics;

public struct WheelHitData : IComponentData
{
    public float3 WheelCenter;
    public float3 HitPoint;
    public float3 HitNormal;
    public float3 VelocityAtContactPoint;

    public float SurfaceFriction;
 
    public bool HasHit;
}
