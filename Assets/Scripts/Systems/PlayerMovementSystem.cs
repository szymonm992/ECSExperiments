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
using ECSExperiment.Wheels;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateAfter(typeof(WheelRaycastSystem))]
public partial struct PlayerMovementSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        state.Dependency.Complete();
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

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

        List<(int rigId, float3 force, float3 point)> impulsePoints = new();
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
            var tireVel = physicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.ValueRO.WheelCenter);

            float steeringVel = math.dot(localRightDirection, tireVel);
            float desiredSidewaysVelChange = -steeringVel * FRICTION_GRIP_FACTOR;
            float desiredSidewaysAccel = desiredSidewaysVelChange / SystemAPI.Time.DeltaTime;

            if (wheelProperties.ValueRO.IsGrounded)
            {
                impulsePoints.Add(new(rigidbodyIndex, desiredSidewaysAccel * localRightDirection, hitData.ValueRO.WheelCenter));
                //physicsWorld.ApplyImpulse(rigidbodyIndex, desiredSidewaysAccel * localRightDirection, hitData.ValueRO.WheelCenter);
            }

            float forwardVel = math.dot(localForwardDirection, tireVel);
            float desiredForwardVelChange = -forwardVel * FRICTION_GRIP_FACTOR;
            float desiredForwardAccel = desiredForwardVelChange / SystemAPI.Time.DeltaTime;

            if (wheelProperties.ValueRO.IsGrounded)
            {
                impulsePoints.Add(new (rigidbodyIndex, desiredForwardAccel * localForwardDirection, hitData.ValueRO.WheelCenter));
                //physicsWorld.ApplyImpulse(rigidbodyIndex, desiredForwardAccel * localForwardDirection, hitData.ValueRO.WheelCenter);
            }


            Debug.DrawRay(hitData.ValueRO.HitPoint, localForwardDirection, Color.blue);
            Debug.DrawRay(hitData.ValueRO.HitPoint, localRightDirection, Color.red);
        }


        foreach (var chuj in impulsePoints)
        {
            physicsWorld.ApplyImpulse(chuj.rigId, chuj.force, chuj.point);
        }
    }
}


