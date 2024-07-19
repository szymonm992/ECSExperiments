using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;

namespace ECSExperiment.Wheels
{
    [BurstCompile]
    [UpdateInGroup(typeof(PhysicsSimulationGroup), OrderFirst = true)]
    public partial struct WheelCastSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency.Complete();

            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            var ecb = new EntityCommandBuffer(Allocator.TempJob);

            foreach (var (wheelProperties, wheelHitData, wheelLocalTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRW<WheelHitData>, RefRO<LocalTransform>>())
            {
                var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);

                if (!IsRigidbodyIndexValid(rigidbodyIndex, physicsWorld))
                {
                    return;
                }

                var wheelGlobalTransform = WheelTransformRelativeToRigidbody(wheelLocalTransform.ValueRO, physicsWorld, rigidbodyIndex);
                var wheelLocalUp = math.mul(wheelGlobalTransform.Rotation, math.up());
                var wheelLocalRight = math.mul(wheelGlobalTransform.Rotation, math.right());

                var castInput = CreateWheelCast(wheelProperties.ValueRO, wheelGlobalTransform.Position, wheelLocalUp);
                var wheelCenter = castInput.Start - (wheelLocalUp * wheelProperties.ValueRO.SpringLength);

                UpdateWheelHitData(ref wheelHitData.ValueRW, false, float3.zero, wheelCenter, physicsWorld.GetLinearVelocity(rigidbodyIndex, wheelCenter), 0f, 0f);

                if (physicsWorld.CollisionWorld.CastRay(castInput, out var result))
                {
                    var raycastDistance = math.distance(result.Position, castInput.Start);
                    var velocityAtWheel = physicsWorld.GetLinearVelocity(rigidbodyIndex, wheelCenter);

                    wheelProperties.ValueRW.IsGrounded = true;
                    wheelCenter = castInput.Start - (wheelLocalUp * raycastDistance);
                    UpdateWheelHitData(ref wheelHitData.ValueRW, true, result.Position, wheelCenter, velocityAtWheel, result.Material.Friction, raycastDistance);

                    var vehicleProperties = SystemAPI.GetComponent<VehicleProperties>(wheelProperties.ValueRO.VehicleEntity);
                    var wheelsCountFraction = 1f / vehicleProperties.WheelsAmount;
                    var currentSpeedUp = math.dot(velocityAtWheel, wheelLocalUp);
                    
                    float3 localVelocityA = currentSpeedUp * wheelLocalUp;
                    float3 localVelocityB = physicsWorld.GetLinearVelocity(result.RigidBodyIndex, wheelCenter);
                    float3 totalSuspensionForce = (wheelProperties.ValueRO.Spring * (result.Position - castInput.End))
                        + (wheelProperties.ValueRO.Damper * (localVelocityB - localVelocityA)) * wheelsCountFraction;

                    float impulseUp = math.dot(totalSuspensionForce, wheelLocalUp);
                    float downForceLimit = -0.25f;

                    if (downForceLimit < impulseUp)
                    {
                        totalSuspensionForce = impulseUp * wheelLocalUp;

                        var vehicleForceAccumulationBuffer = SystemAPI.GetBuffer<ForceAccumulationBufferElement>(wheelProperties.ValueRO.VehicleEntity);
                        vehicleForceAccumulationBuffer.Add(new()
                        {
                            point = wheelCenter,
                            force = totalSuspensionForce
                        });
                    }
                }
                else
                {
                    wheelProperties.ValueRW.IsGrounded = false;
                }

                #if UNITY_EDITOR
                float3 localRightDirection = math.mul(wheelGlobalTransform.Rotation,
                    new float3((int)wheelProperties.ValueRO.Side, 0, 0));

                Debug.DrawRay(castInput.Start, castInput.End - castInput.Start, wheelProperties.ValueRO.IsGrounded ? Color.green : Color.red);
                Debug.DrawRay(wheelCenter, localRightDirection, Color.red);
                #endif
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        [BurstCompile]
        private RaycastInput CreateWheelCast(in WheelProperties wheelProperties, float3 wheelGlobalPosition, float3 wheelLocalUp)
        {
            var rayStart = wheelGlobalPosition + (wheelLocalUp * wheelProperties.Radius);
            var rayEnd = rayStart - wheelLocalUp * (wheelProperties.SpringLength + wheelProperties.Radius);

            return new RaycastInput
            {
                Start = rayStart,
                End = rayEnd,
                Filter = CollisionFilter.Default,
            };
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
        private void UpdateWheelHitData(ref WheelHitData hitData, bool hit, float3 hitPoint, float3 wheelCenter, float3 velocityAtContactPoint, float surfaceFriction, float hitDistance)
        {
            hitData.HasHit = hit;
            hitData.HitPoint = hitPoint;
            hitData.WheelCenter = wheelCenter;
            hitData.SurfaceFriction = surfaceFriction;
            hitData.VelocityAtContactPoint = velocityAtContactPoint;
            hitData.Distance = hitDistance;
        }

        [BurstCompile]
        private bool IsRigidbodyIndexValid(int rigidbodyIndex, PhysicsWorld physicsWorld)
        {
            return rigidbodyIndex > -1 && rigidbodyIndex < physicsWorld.NumDynamicBodies;
        }
    }
}


