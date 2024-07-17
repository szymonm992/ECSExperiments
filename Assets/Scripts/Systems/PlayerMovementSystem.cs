using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;
using System.Collections.Generic;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
public partial struct PlayerMovementSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

<<<<<<< Updated upstream
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
=======
        /*foreach (var (wheelProperties, hitData, wheelTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRO<WheelHitData>, RefRO<LocalToWorld>>())
        {  
            var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);

            if (wheelProperties.ValueRO.IsGrounded)
            {
                wheelProperties.ValueRW.compression = 1.0f - ((hitData.ValueRO.Distance - wheelProperties.ValueRO.Radius) / wheelProperties.ValueRO.SpringLength);
                wheelProperties.ValueRW.wheelVelo = physicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.ValueRO.WheelCenter);
                wheelProperties.ValueRW.localVelo = math.mul(wheelProperties.ValueRO.inverseLocalRotation, wheelProperties.ValueRW.wheelVelo);

                float normalForce = wheelProperties.ValueRW.compression * wheelProperties.ValueRW.fullCompressionSpringForce;
                wheelProperties.ValueRW.roadForce = RoadForce(ref state, wheelProperties, hitData, normalForce, wheelTransform.ValueRO);

                physicsWorld.ApplyImpulse(rigidbodyIndex, wheelProperties.ValueRW.roadForce, hitData.ValueRO.WheelCenter);
            }
            else
            {
                wheelProperties.ValueRW.compression = 0.0f;
                wheelProperties.ValueRW.roadForce = float3.zero;

                float totalInertia = wheelProperties.ValueRO.inertia + wheelProperties.ValueRO.drivetrainInertia;
                float driveAngularDelta = wheelProperties.ValueRO.driveTorque * SystemAPI.Time.DeltaTime / totalInertia;
                float totalFrictionTorque = wheelProperties.ValueRO.brakeFrictionTorque * wheelProperties.ValueRO.brake +
                    wheelProperties.ValueRO.handbrakeFrictionTorque * wheelProperties.ValueRO.handbrake + wheelProperties.ValueRO.frictionTorque + wheelProperties.ValueRO.driveFrictionTorque;
                float frictionAngularDelta = totalFrictionTorque * SystemAPI.Time.DeltaTime / totalInertia;

                wheelProperties.ValueRW.angularVelocity += driveAngularDelta;

                if (math.abs(wheelProperties.ValueRW.angularVelocity) > frictionAngularDelta)
                {
                    wheelProperties.ValueRW.angularVelocity -= frictionAngularDelta * math.sign(wheelProperties.ValueRW.angularVelocity);
                }
                else
                {
                    wheelProperties.ValueRW.angularVelocity = 0;
                }

                wheelProperties.ValueRW.slipRatio = 0;
                wheelProperties.ValueRW.slipVelo = 0;
            }

            wheelProperties.ValueRW.compression = Mathf.Clamp01(wheelProperties.ValueRW.compression);
            */


        foreach (var (wheelProperties, hitData, wheelLocalTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRO<WheelHitData>, RefRO<LocalTransform>>())
        {
            var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);

            var vehicleRigidbodyTransform = LocalTransform.FromMatrix(float4x4.TRS(
                    PhysicsWorldExtensions.GetPosition(physicsWorld, rigidbodyIndex),
                    PhysicsWorldExtensions.GetRotation(physicsWorld, rigidbodyIndex),
                    new float3(1f, 1f, 1f)
                ));

            var wheelCastOriginGlobalTransform = vehicleRigidbodyTransform.TransformTransform(wheelLocalTransform.ValueRO);

            float FRICTION_GRIP_FACTOR = 1f;

            float3 localRightDirection = math.mul(wheelCastOriginGlobalTransform.Rotation, math.right());
            float3 localForwardDirection = math.forward(wheelCastOriginGlobalTransform.Rotation);
            float3 springDir = math.mul(wheelCastOriginGlobalTransform.Rotation, math.up());
            var tireVel = physicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.ValueRO.WheelCenter);

            float steeringVel = math.dot(localRightDirection, tireVel);
            float desiredSidewaysVelChange = -steeringVel * FRICTION_GRIP_FACTOR;
            float desiredSidewaysAccel = desiredSidewaysVelChange / SystemAPI.Time.DeltaTime;

            if (wheelProperties.ValueRO.IsGrounded)
            {
                
                physicsWorld.ApplyImpulse(rigidbodyIndex, desiredSidewaysAccel * localRightDirection, hitData.ValueRO.WheelCenter);
            }

            float forwardVel = math.dot(localForwardDirection, tireVel);
            float desiredForwardVelChange = -forwardVel * FRICTION_GRIP_FACTOR;
            float desiredForwardAccel = desiredForwardVelChange / SystemAPI.Time.DeltaTime;

            if (wheelProperties.ValueRO.IsGrounded)
            {
                
                physicsWorld.ApplyImpulse(rigidbodyIndex, desiredForwardAccel * localForwardDirection, hitData.ValueRO.WheelCenter);
            }


            Debug.DrawRay(hitData.ValueRO.HitPoint, localForwardDirection, Color.blue);
            Debug.DrawRay(hitData.ValueRO.HitPoint, localRightDirection, Color.red);
>>>>>>> Stashed changes
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



