using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;
using Unity.Collections;

namespace ECSExperiment.Wheels
{
    [BurstCompile]
    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateAfter(typeof(WheelCastSystem))]
    public partial struct VehicleFrictionSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency.Complete();

            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            var ecb = new EntityCommandBuffer(Allocator.TempJob);

            foreach (var (wheelProperties, hitData, wheelLocalTransform) in SystemAPI.Query<RefRO<WheelProperties>, RefRO<WheelHitData>, RefRO<LocalTransform>>())
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
                float3 tireVel = physicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.ValueRO.WheelCenter);

                float steeringVel = math.dot(localRightDirection, tireVel);
                float desiredSidewaysVelChange = -steeringVel * FRICTION_GRIP_FACTOR;
                float desiredSidewaysAccel = desiredSidewaysVelChange / SystemAPI.Time.DeltaTime;

                float forwardVel = math.dot(localForwardDirection, tireVel);
                float desiredForwardVelChange = -forwardVel * FRICTION_GRIP_FACTOR;
                float desiredForwardAccel = desiredForwardVelChange / SystemAPI.Time.DeltaTime;

                if (wheelProperties.ValueRO.IsGrounded)
                {
                    //RequestForceAccumulation(ref state, wheelProperties.ValueRO.VehicleEntity, desiredSidewaysAccel * localRightDirection, hitData.ValueRO.WheelCenter);
                    //RequestForceAccumulation(ref state, wheelProperties.ValueRO.VehicleEntity, desiredForwardAccel * localForwardDirection, hitData.ValueRO.WheelCenter);
                }

                //Debug.DrawRay(hitData.ValueRO.HitPoint, localForwardDirection, Color.blue);
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        [BurstCompile]
        private void RequestForceAccumulation(ref SystemState state, Entity bufferEntity, float3 appliedForce, float3 appliedForcePoint)
        {
            var vehicleForceAccumulationBuffer = SystemAPI.GetBuffer<ForceAccumulationBufferElement>(bufferEntity);
            vehicleForceAccumulationBuffer.Add(new()
            {
                force = appliedForce,
                point = appliedForcePoint,
            });
        }

        [BurstCompile]
        private LocalTransform WheelTransformRelativeToRigidbody(LocalTransform wheelLocalTransform, PhysicsWorld physicsWorld, int rigidbodyIndex)
        {
            if (!IsRigidbodyIndexValid(rigidbodyIndex, physicsWorld))
            {
                return default;
            }

            var vehicleRigidbodyTransform = LocalTransform.FromMatrix(float4x4.TRS(
                        PhysicsWorldExtensions.GetPosition(physicsWorld, rigidbodyIndex),
                        PhysicsWorldExtensions.GetRotation(physicsWorld, rigidbodyIndex),
                        new float3(1f, 1f, 1f)
                    ));

            return vehicleRigidbodyTransform.TransformTransform(wheelLocalTransform);
        }

        [BurstCompile]
        private bool IsRigidbodyIndexValid(int rigidbodyIndex, PhysicsWorld physicsWorld)
        {
            return rigidbodyIndex > -1 && rigidbodyIndex < physicsWorld.NumDynamicBodies;
        }
    }
}


