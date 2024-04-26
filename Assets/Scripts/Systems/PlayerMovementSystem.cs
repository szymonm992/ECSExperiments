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

        foreach (var (inputs, properties) in SystemAPI.Query<RefRO<InputsData>, RefRO<VehicleProperties>>())
        {
            var vehicleDriveJob = new DriveJob
            {
                PhysicsWorld = physicsWorld,
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                InputsData = inputs.ValueRO,
                VehicleProperties = properties.ValueRO,
                FixedTime = SystemAPI.Time.fixedDeltaTime,
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
        LocalTransform vehicleTransform = LocalTransformLookup[wheelProperties.VehicleEntity];

        float3 worldUpDirection = new float3(0, 1, 0);
        //float3 localUpDirection = math.mul(vehicleTransform.Rotation, worldUpDirection);
        float3 gravityImpulse = -9.81f * worldUpDirection;
        PhysicsWorld.ApplyImpulse(rigidbodyIndex, gravityImpulse, vehicleTransform.Position);

        if (!wheelProperties.IsGrounded || !wheelProperties.CanDrive)
        {
            return;
        }

        float3 worldForwardDirection = new float3(0, 0, 1);
        float3 localForwardDirection = math.mul(vehicleTransform.Rotation, worldForwardDirection);

        if (VehicleProperties.CurrentSpeed <= currentMaxSpeed)
        {
            float impulsePower = InputsData.Vertical * (currentMaxSpeed * 0.5f);
            float3 impulse = impulsePower * localForwardDirection;

            PhysicsWorld.ApplyImpulse(rigidbodyIndex, impulse, hitData.HitPoint);
        }

        float FRICTION_GRIP_FACTOR = 1f;
        float3 localRightDirection = math.mul(vehicleTransform.Rotation, new float3(-1, 0, 0));

        var tireVel = PhysicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.HitPoint);
        
        float steeringVel = math.dot(localRightDirection, tireVel);
        float desiredSidewaysVelChange = -steeringVel * FRICTION_GRIP_FACTOR;
        float desiredSidewaysAccel = desiredSidewaysVelChange / FixedTime;

        PhysicsWorld.ApplyImpulse(rigidbodyIndex, desiredSidewaysAccel * localRightDirection, hitData.HitPoint);


        tireVel = PhysicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.HitPoint);

        float forwardVel = math.dot(localForwardDirection, tireVel);
        float desiredForwardVelChange = -forwardVel * FRICTION_GRIP_FACTOR;
        float desiredForwardAccel = desiredForwardVelChange / FixedTime;

        PhysicsWorld.ApplyImpulse(rigidbodyIndex, desiredForwardAccel * localForwardDirection, hitData.HitPoint);

        Debug.DrawRay(hitData.HitPoint, localForwardDirection, Color.blue);
        Debug.DrawRay(hitData.HitPoint, localRightDirection, Color.red);

    }
}



