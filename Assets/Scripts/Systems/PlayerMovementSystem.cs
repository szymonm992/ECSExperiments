using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
public partial struct PlayerMovementSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

        foreach (var (inputs, properties) in SystemAPI.Query<RefRO<InputsData>, RefRO<VehicleProperties>>())
        {
            var vehicleDriveJob = new DriveJob
            {
                PhysicsWorld = physicsWorld,
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                InputsData = inputs.ValueRO,
                VehicleProperties = properties.ValueRO,
                FixedTime = SystemAPI.Time.DeltaTime,
            };
            state.Dependency = vehicleDriveJob.Schedule(state.Dependency);
        }
    }
}

[BurstCompile]
[WithAll(typeof(Simulate))]
public partial struct DriveJob : IJobEntity
{
    [ReadOnly] public ComponentLookup<LocalTransform> LocalTransformLookup;
    [ReadOnly] public InputsData InputsData;
    [ReadOnly] public VehicleProperties VehicleProperties;

    public PhysicsWorld PhysicsWorld;
    public float FixedTime;

    private void Execute(in WheelProperties wheelProperties, in WheelHitData hitData)
    {
        bool isInputPositive = InputsData.Vertical.IsNumberPositive();

        var rigidbodyIndex = PhysicsWorld.GetRigidBodyIndex(wheelProperties.VehicleEntity);
        var currentMaxSpeed = isInputPositive ? VehicleProperties.VehicleMaximumForwardSpeed : VehicleProperties.VehicleMaximumBackwardSpeed;

        if (!wheelProperties.IsGrounded || !wheelProperties.CanDrive)
        {
            return;
        }

        if (VehicleProperties.CurrentSpeed <= currentMaxSpeed)
        {
            LocalTransform vehicleTransform = LocalTransformLookup[wheelProperties.VehicleEntity];

            float3 worldForwardDirection = new float3(0, 0, 1);
            float3 localForwardDirection = math.mul(vehicleTransform.Rotation, worldForwardDirection);

            float impulsePower = InputsData.Vertical * (currentMaxSpeed * 0.5f);
            float3 impulse = impulsePower * localForwardDirection;

            PhysicsWorld.ApplyImpulse(rigidbodyIndex, impulse, hitData.HitPoint);
        }
    }
}



