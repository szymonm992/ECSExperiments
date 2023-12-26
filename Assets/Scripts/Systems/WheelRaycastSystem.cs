using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Entities.UniversalDelegates;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateBefore(typeof(PlayerMovementSystem))]
public partial struct WheelRaycastSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
        var wheelRaycastJob = new WheelRaycastJob()
        {
            PhysicsWorld = physicsWorld,
            LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true)
        };

        state.Dependency = wheelRaycastJob.Schedule(state.Dependency);  
    }
}

[BurstCompile]
[WithAll(typeof(Simulate))]
public partial struct WheelRaycastJob : IJobEntity
{ 
    public PhysicsWorld PhysicsWorld;
    [ReadOnly] public ComponentLookup<LocalTransform> LocalTransformLookup;

    private void Execute(Entity entity, in VehicleEntityProperties vehicleEntityProperties, in WheelProperties wheelProperties)
    {
        var wheelTransform = LocalTransformLookup[entity];
        var vehicleTransform = LocalTransformLookup[vehicleEntityProperties.VehicleEntity];

        var localPosition = math.mul(vehicleTransform.Rotation, wheelTransform.Position) + vehicleTransform.Position;
        var localUp = math.mul(vehicleTransform.Rotation, new float3(0, 1, 0));

        var rigIndex = PhysicsWorld.GetRigidBodyIndex(vehicleEntityProperties.VehicleEntity);
        var collisionFilter = PhysicsWorld.GetCollisionFilter(rigIndex);

        var rayStart = localPosition;
        var rayEnd = rayStart - localUp * (wheelProperties.Radius * wheelProperties.Travel);

        var raycastInput = new RaycastInput()
        {
            Start = rayStart,
            End = rayEnd,
            Filter = collisionFilter,
        };

        if (PhysicsWorld.CastRay(raycastInput, out RaycastHit result))
        {

        }
    }
}
