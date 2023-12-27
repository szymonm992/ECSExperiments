using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
public partial struct PlayerMovementSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
        
        foreach (var (inputs, properties) in SystemAPI.Query<RefRO<InputsData>, RefRW<VehicleProperties>>())
        {
            foreach (var (wheel, hitData) in SystemAPI.Query<RefRO<WheelProperties>, RefRO<WheelHitData>>())
            {
                NativeArray<float> returnedValues = new (2, Allocator.TempJob);
                var vehicleDriveJob = new DriveJob
                {
                    PhysicsWorld = physicsWorld,
                    LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                    Inputs = inputs.ValueRO,
                    VehicleProperties = properties.ValueRO,
                    ReturnedValues = returnedValues,
                };

                properties.ValueRW.CurrentSpeed = returnedValues[0];
                state.Dependency = vehicleDriveJob.Schedule(state.Dependency);
            }
        }
    }

    [BurstCompile]
    [WithAll(typeof(Simulate))]
    public partial struct DriveJob : IJobEntity
    {
        [ReadOnly] public ComponentLookup<LocalTransform> LocalTransformLookup;
        [ReadOnly] public VehicleProperties VehicleProperties;
        [ReadOnly] public InputsData Inputs;

        public PhysicsWorld PhysicsWorld;
        public NativeArray<float> ReturnedValues;

        private void Execute(in WheelProperties wheelProperties, in WheelHitData hitData)
        {
            bool isInputPositive = Inputs.Vertical.IsNumberPositive();
            var rigidbodyIndex = PhysicsWorld.GetRigidBodyIndex(VehicleProperties.VehicleEntity);

            var currentVelocity = PhysicsWorld.GetLinearVelocity(rigidbodyIndex);
            var currentMaxSpeed = isInputPositive ? VehicleProperties.VehicleMaximumForwardSpeed : VehicleProperties.VehicleMaximumBackwardSpeed;
            var currentSpeed = math.length(currentVelocity) * 4f;

            ReturnedValues[0] = currentSpeed;

            if (!wheelProperties.IsGrounded || !wheelProperties.CanDrive)
            {
                return;
            }

            if (currentSpeed <= currentMaxSpeed)
            {
                LocalTransform vehicleTransform = LocalTransformLookup[VehicleProperties.VehicleEntity];

                float3 worldForwardDirection = new float3(0, 0, 1);
                float3 localForwardDirection = math.mul(vehicleTransform.Rotation, worldForwardDirection);

                float impulsePower = Inputs.Vertical * (currentMaxSpeed * 0.5f);
                float3 impulse = impulsePower * localForwardDirection;

                PhysicsWorld.ApplyImpulse(rigidbodyIndex, impulse, hitData.HitPoint);
            }
        }
    }
}


