using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.UIElements;

[UpdateInGroup(typeof(PhysicsSystemGroup))]
[UpdateAfter(typeof(PhysicsInitializeGroup)), UpdateBefore(typeof(PhysicsSimulationGroup))]
[BurstCompile]
public partial struct PlayerMechanicsSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        new PrepareVehicleJob { }.Schedule();
        state.Dependency.Complete();

        PhysicsWorld physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
        state.EntityManager.CompleteDependencyBeforeRW<PhysicsWorldSingleton>();

        var entityCommandBuffer = new EntityCommandBuffer(Unity.Collections.Allocator.TempJob);

        foreach(var (localTransform, wheel, entity) in SystemAPI.Query<RefRW<LocalTransform>, RefRO<WheelData>>().WithEntityAccess())
        {
            var newLocalTransform = localTransform;

            Entity vehicleEntity = wheel.ValueRO.VehicleEntity;
            if (vehicleEntity == null)
            {
                Debug.Log($"Vehicle entity of {wheel.ValueRO.ToString()} is null!");
                return;
            }

            int vehicleEntityIndex = physicsWorld.GetRigidBodyIndex(vehicleEntity);
            if (vehicleEntityIndex == -1 || vehicleEntityIndex >= physicsWorld.NumDynamicBodies ) 
            {
                Debug.Log($"Vehicle rigidbody index is not in valid range! Index number: {vehicleEntityIndex}");
                return;
            }

            var vehicleBody = SystemAPI.GetComponent<VehicleBody>(vehicleEntity);
            var entityTransform = SystemAPI.GetComponent<LocalTransform>(vehicleEntity);

            float3 vehicleEntityPosition = entityTransform.Position;
            float3 vehicleEntityCOM = vehicleBody.WorldCenterOfMass;
            quaternion vehicleEntityRotation = entityTransform.Rotation;

            float3 entityTransformUp = math.mul(vehicleEntityRotation, new float3(0f, 1f, 0f));
            float3 entityTransformForward = math.mul(vehicleEntityRotation, new float3(0f, 0f, 1f));
            float3 entityTransformRight = math.mul(vehicleEntityRotation, new float3(1f, 0f, 0f));


            CollisionFilter collisionFilter = physicsWorld.GetCollisionFilter(vehicleEntityIndex);

            RigidTransform worldFromChassis = new RigidTransform
            {
                pos = vehicleEntityPosition,
                rot = vehicleEntityRotation
            };

            RigidTransform suspensionFromWheel = new RigidTransform
            {
                pos = localTransform.ValueRO.Position,
                rot = localTransform.ValueRO.Rotation
            };

            RigidTransform chassisFromWheel = math.mul(wheel.ValueRO.ChassisFromSuspension, suspensionFromWheel);
            RigidTransform worldFromLocal = math.mul(worldFromChassis, chassisFromWheel);

            // create a raycast from the suspension point on the chassis
            var worldFromSuspension = math.mul(worldFromChassis, wheel.ValueRO.ChassisFromSuspension);
        }
    }

    [BurstCompile]
    partial struct PrepareVehicleJob : IJobEntity
    {
        public readonly void Execute(ref VehicleBody body, in PhysicsMass mass, in LocalTransform localTransform)
        {
            body.WorldCenterOfMass = mass.GetCenterOfMassWorldSpace(localTransform.Position, localTransform.Rotation);

            float3 worldUpDirection = math.mul(localTransform.Rotation, math.up());
            body.SlopeSlipFactor = math.pow(math.abs(math.dot(worldUpDirection, math.up())), 4f);
        }
    }
}
