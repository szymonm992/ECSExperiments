using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;

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

                var wheelGlobalTransform = GetWorldTransformRelativeToRigidbody(wheelLocalTransform.ValueRO, physicsWorld, rigidbodyIndex);
                var wheelLocalUp = math.mul(wheelGlobalTransform.Rotation, math.up());
                var wheelLocalRight = math.mul(wheelGlobalTransform.Rotation, math.right());

                var castInput = CreateWheelCast(wheelProperties.ValueRO, wheelGlobalTransform.Position, wheelLocalUp);
                var wheelCenter = castInput.Start - (wheelLocalUp * wheelProperties.ValueRO.SpringLength);

                UpdateWheelHitData(ref wheelHitData.ValueRW, false, float3.zero, wheelCenter, physicsWorld.GetLinearVelocity(rigidbodyIndex, wheelCenter), math.up(), 0f);

                if (physicsWorld.CollisionWorld.CastRay(castInput, out var result))
                {
                    var raycastDistance = math.distance(result.Position, castInput.Start);
                    var rayMaxDistance = math.distance(castInput.Start, castInput.End);

                    var velocityAtWheel = physicsWorld.GetLinearVelocity(rigidbodyIndex, wheelCenter);
                    var compression = 1f - (raycastDistance / rayMaxDistance);

                    wheelProperties.ValueRW.Compression = compression;
                    wheelProperties.ValueRW.IsGrounded = true;

                    wheelCenter = castInput.Start - (wheelLocalUp * (raycastDistance - wheelProperties.ValueRO.Radius));
                    UpdateWheelHitData(ref wheelHitData.ValueRW, true, result.Position, wheelCenter, velocityAtWheel, result.SurfaceNormal, result.Material.Friction);

                    var currentSpeedUp = math.dot(velocityAtWheel, wheelLocalUp);
                    
                    float3 localVelocityOfVehicle = currentSpeedUp * wheelLocalUp;
                    float3 localVelocityOfHitBody = physicsWorld.GetLinearVelocity(result.RigidBodyIndex, wheelCenter);
                    float3 totalSuspensionForce = (wheelProperties.ValueRO.Spring * (result.Position - castInput.End))
                        + (wheelProperties.ValueRO.Damper * (localVelocityOfHitBody - localVelocityOfVehicle)) * wheelProperties.ValueRO.WheelsAmountFraction;

                    float suspensionForce = math.dot(totalSuspensionForce, wheelLocalUp);
                    totalSuspensionForce = suspensionForce * wheelLocalUp;

                    Debug.Log($"Compression is {compression}");
                    RequestForceAccumulation(ref state, wheelProperties.ValueRO.VehicleEntity, totalSuspensionForce, wheelCenter);                 
                }
                else
                {
                    wheelProperties.ValueRW.IsGrounded = false;
                    wheelProperties.ValueRW.Compression = 0f;
                }

                RepositionVisualWheel(ref state, ecb, wheelGlobalTransform, wheelProperties.ValueRO.WheelVisualObjectEntity, wheelCenter);

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
        private LocalTransform GetWorldTransformRelativeToRigidbody(LocalTransform wheelLocalTransform, PhysicsWorld physicsWorld, int rigidbodyIndex)
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
        private void UpdateWheelHitData(ref WheelHitData hitData, bool hit, float3 hitPoint, float3 wheelCenter, float3 velocityAtContactPoint, float3 normal, float surfaceFriction)
        {
            hitData.HasHit = hit;
            hitData.HitPoint = hitPoint;
            hitData.WheelCenter = wheelCenter;
            hitData.SurfaceFriction = surfaceFriction;
            hitData.VelocityAtContactPoint = velocityAtContactPoint;
            hitData.HitNormal = normal;
        }

        [BurstCompile]
        private bool IsRigidbodyIndexValid(int rigidbodyIndex, PhysicsWorld physicsWorld)
        {
            return rigidbodyIndex > -1 && rigidbodyIndex < physicsWorld.NumDynamicBodies;
        }

        [BurstCompile]
        private void RequestForceAccumulation(ref SystemState _, Entity bufferEntity, float3 appliedForce, float3 appliedForcePoint)
        {
            var vehicleForceAccumulationBuffer = SystemAPI.GetBuffer<ForceAccumulationBufferElement>(bufferEntity);
            vehicleForceAccumulationBuffer.Add(new()
            {
                force = appliedForce,
                point = appliedForcePoint,
            });
        }

        [BurstCompile]
        private void RepositionVisualWheel(ref SystemState _, EntityCommandBuffer ecb,LocalTransform parentTransform, Entity visualWheelEntity, float3 wheelCenter)
        {
            var wheelLocalTransform = GetLocalTransformRelativeToParent(parentTransform, wheelCenter);
            ecb.SetComponent(visualWheelEntity, wheelLocalTransform);
        }

        [BurstCompile]
        private LocalTransform GetLocalTransformRelativeToParent(LocalTransform parentTransform, float3 worldPosition)
        {
            var parentMatrix = parentTransform.ToMatrix();
            var inverseParentMatrix = math.inverse(parentMatrix);

            var localPosition = math.mul(inverseParentMatrix, new float4(worldPosition, 1f)).xyz;

            return new LocalTransform
            {
                Position = localPosition,
                Rotation = parentTransform.Rotation,
                Scale = 1f
            };
        }
    }
}


