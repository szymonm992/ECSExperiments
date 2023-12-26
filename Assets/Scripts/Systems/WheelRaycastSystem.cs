using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.SocialPlatforms;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateBefore(typeof(PlayerMovementSystem))]
public partial struct WheelRaycastSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<VehicleEntityProperties>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

        foreach (var vehicleEntityProperties in SystemAPI.Query<RefRO<VehicleEntityProperties>>())
        {
            foreach (var (wheel, hitData, suspensionData) in SystemAPI.Query<RefRO<WheelProperties>, RefRW<WheelHitData>, RefRO<Suspension>>())
            {
                var wheelRaycastJob = new WheelRaycastJob()
                {
                    PhysicsWorld = physicsWorld,
                    LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                    WorldTransformLookup = SystemAPI.GetComponentLookup<LocalToWorld>(true),
                    VehicleEntityProperties = vehicleEntityProperties.ValueRO,
                };

                state.Dependency = wheelRaycastJob.Schedule(state.Dependency);
            }
        }
    }
}

[BurstCompile]
[WithAll(typeof(Simulate))]
public partial struct WheelRaycastJob : IJobEntity
{ 
    public PhysicsWorld PhysicsWorld;
    public VehicleEntityProperties VehicleEntityProperties;
    [ReadOnly] public ComponentLookup<LocalTransform> LocalTransformLookup;
    [ReadOnly] public ComponentLookup<LocalToWorld> WorldTransformLookup;

    private void Execute(in WheelProperties wheelProperties, in Suspension suspension, ref WheelHitData wheelHitData)
    {
        var wheelTransform = WorldTransformLookup[wheelProperties.Entity];
        var vehicleTransform = LocalTransformLookup[VehicleEntityProperties.VehicleEntity];

        float3 worldUpDirection = new float3(0, 1, 0);
        float3 localUpDirection = math.mul(vehicleTransform.Rotation, worldUpDirection);

        var rigidbodyIndex = PhysicsWorld.GetRigidBodyIndex(VehicleEntityProperties.VehicleEntity);
        var rayStart = wheelTransform.Position + (localUpDirection * wheelProperties.Radius);
        var rayEnd = rayStart - localUpDirection * (wheelProperties.Travel + wheelProperties.Radius);
        var filter = PhysicsWorld.GetCollisionFilter(rigidbodyIndex);

        var raycastInput = new RaycastInput
        {
            Start = rayStart,
            End = rayEnd,
            Filter = filter
        };

        bool hit = PhysicsWorld.CollisionWorld.CastRay(raycastInput, out var result);

        #if UNITY_EDITOR
        Color rayColor = hit ? Color.green : Color.red;
        Debug.DrawRay(rayStart, rayEnd - rayStart, rayColor);
        #endif

        if (hit)
        {
            float3 impulsePoint = PhysicsWorld.Bodies[rigidbodyIndex].WorldFromBody.pos;
            PhysicsWorld.ApplyImpulse(rigidbodyIndex, 30 * localUpDirection, impulsePoint);
        }

    }

}
