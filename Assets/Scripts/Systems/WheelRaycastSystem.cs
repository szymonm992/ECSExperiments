using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
public partial struct WheelRaycastSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

        foreach (var properties in SystemAPI.Query<RefRW<VehicleEntityProperties>>())
        {
            foreach (var (wheel, hitData) in SystemAPI.Query<RefRO<WheelProperties>, RefRW<WheelHitData>>())
            {
                var wheelRaycastJob = new WheelRaycastJob()
                {
                    PhysicsWorld = physicsWorld,
                    LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true)
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
    [ReadOnly] public ComponentLookup<LocalTransform> LocalTransformLookup;

    private void Execute(in VehicleEntityProperties vehicleEntityProperties,
        in WheelProperties wheelProperties,  ref WheelHitData wheelHitData)
    {
        var vehicleTransform = LocalTransformLookup[vehicleEntityProperties.VehicleEntity];
        var rigidbodyIndex = PhysicsWorld.GetRigidBodyIndex(vehicleEntityProperties.VehicleEntity);
        var localUpDirection = math.mul(vehicleTransform.Rotation, new float3(0, 1, 0));

        float3 impulsePoint = PhysicsWorld.Bodies[rigidbodyIndex].WorldFromBody.pos;
        PhysicsWorld.ApplyImpulse(rigidbodyIndex, 10000 * localUpDirection, impulsePoint);

        /*
        var wheelTransform = LocalTransformLookup[entity];
       

        var localPosition = math.mul(vehicleTransform.Rotation, wheelTransform.Position) + vehicleTransform.Position;
       
        
        var collisionFilter = PhysicsWorld.GetCollisionFilter(rigidbodyIndex);

        var rayStart = localPosition;
        //var rayEnd = rayStart - localUpDirection * (suspension.RestLength + wheelProperties.Radius);
        var rayEnd = rayStart - localUpDirection * (2f + wheelProperties.Radius);

        var raycastInput = new RaycastInput()
        {
            Start = rayStart,
            End = rayEnd,
            Filter = collisionFilter,
        };

        wheelHitData.WheelCenter = localPosition - localUpDirection * suspension.SpringLength;
        wheelHitData.Velocity = PhysicsWorld.GetLinearVelocity(rigidbodyIndex, wheelHitData.WheelCenter);
        wheelHitData.HasHit = false;

        if (math.length(wheelHitData.Velocity) > 50)
        {
            wheelHitData.Reset();
            return;
        }

        if (PhysicsWorld.CastRay(raycastInput, out RaycastHit result))
        {
            wheelHitData.Origin = localPosition;
            wheelHitData.HasHit = true;
            wheelHitData.Position = result.Position;
            wheelHitData.SurfaceFriction = result.Material.Friction;

            PhysicsWorld.ApplyImpulse(rigidbodyIndex, 10000 * localUpDirection, 
                localPosition - localUpDirection * suspension.SpringLength);
        }
        else
        {
            float3 impulsePoint = PhysicsWorld.Bodies[rigidbodyIndex].WorldFromBody.pos;
            PhysicsWorld.ApplyImpulse(rigidbodyIndex, 10000 * localUpDirection, impulsePoint);
        }*/
    }

}
