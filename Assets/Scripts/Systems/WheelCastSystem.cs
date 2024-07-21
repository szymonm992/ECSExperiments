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
    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
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

                var wheelOriginPosition = GetWorldTransformRelativeToRigidbody(wheelLocalTransform.ValueRO, physicsWorld, rigidbodyIndex);
                var wheelLocalUp = math.mul(wheelOriginPosition.Rotation, math.up());
                var wheelLocalRight = math.mul(wheelOriginPosition.Rotation, math.right());

                var castInput = CreateWheelCast(wheelProperties.ValueRO, wheelOriginPosition.Position, wheelLocalUp);
                //rest point is in the middle of spring 
            
                var restPoint = castInput.Start - (wheelLocalUp * (wheelProperties.ValueRO.Radius + (wheelProperties.ValueRO.SpringLength * 0.5f)));
                var wheelCenter = restPoint;
                var lowerConstraintPoint = castInput.Start - (wheelLocalUp * (wheelProperties.ValueRO.Radius + wheelProperties.ValueRO.SpringLength));

               UpdateWheelHitData(ref wheelHitData.ValueRW, false, float3.zero, wheelCenter, physicsWorld.GetLinearVelocity(rigidbodyIndex, wheelCenter), math.up(), 0f);

                if (physicsWorld.CollisionWorld.CastRay(castInput, out var result))
                {
                    var hitDistance = math.distance(result.Position, castInput.Start);
                    wheelCenter = castInput.Start - (wheelLocalUp * (hitDistance - wheelProperties.ValueRO.Radius));

                    var velocityAtWheel = physicsWorld.GetLinearVelocity(rigidbodyIndex, result.Position);
                    var maxCompressedToWheelCenterPositionDistance = math.distance(wheelOriginPosition.Position, wheelCenter);
                    var compression = math.clamp(1f - (maxCompressedToWheelCenterPositionDistance / wheelProperties.ValueRO.SpringLength), 0f, 1f);
                    var distanceToRestPoint = math.distance(castInput.Start, restPoint);

                    wheelProperties.ValueRW.Compression = compression;
                    wheelProperties.ValueRW.IsGrounded = true;

                    UpdateWheelHitData(ref wheelHitData.ValueRW, true, result.Position, wheelCenter, velocityAtWheel, result.SurfaceNormal, result.Material.Friction);
                    
                    float offset = (distanceToRestPoint - hitDistance + wheelProperties.ValueRO.Radius) / (wheelProperties.ValueRO.SpringLength * 0.5f);
                    float velocity = math.dot(wheelLocalUp, velocityAtWheel);

                    float normalForce = (offset * wheelProperties.ValueRO.Spring) - (velocity * wheelProperties.ValueRO.Damper);
                    Debug.Log($"Offset is equal {offset} Current conmpression is {compression}");
                    RequestForceAccumulation(ref state, wheelProperties.ValueRO.VehicleEntity, wheelLocalUp * normalForce, result.Position);

                    Debug.DrawRay(result.Position, wheelLocalUp * normalForce, Color.blue);
                }
                else
                {
                    wheelProperties.ValueRW.IsGrounded = false;
                    wheelProperties.ValueRW.Compression = 0f;
                }

                RepositionVisualWheel(ref state, ecb, wheelOriginPosition, wheelProperties.ValueRO.WheelVisualObjectEntity, wheelCenter);

                #if UNITY_EDITOR
                float3 localRightDirection = math.mul(wheelOriginPosition.Rotation,
                    new float3((int)wheelProperties.ValueRO.Side, 0, 0));

                Debug.DrawRay(castInput.Start, castInput.End - castInput.Start, wheelProperties.ValueRO.IsGrounded ? Color.green : Color.red);
                Debug.DrawRay(wheelCenter, localRightDirection, Color.white);

                Debug.DrawRay(restPoint, wheelLocalRight, Color.yellow);
                Debug.DrawRay(wheelOriginPosition.Position, wheelLocalRight, Color.cyan);
                Debug.DrawRay(lowerConstraintPoint, wheelLocalRight, Color.cyan);

               
                #endif
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        [BurstCompile]
        private RaycastInput CreateWheelCast(in WheelProperties wheelProperties, float3 wheelGlobalPosition, float3 wheelLocalUp)
        {
            var rayStart = wheelGlobalPosition + (wheelLocalUp * wheelProperties.Radius);
            var rayEnd = rayStart - wheelLocalUp * (wheelProperties.SpringLength + (wheelProperties.Radius * 2f));

            return new RaycastInput
            {
                Start = rayStart,
                End = rayEnd,
                Filter = CollisionFilter.Default,
            };
        }

        public float Clamp01(float value)
        {
            return math.clamp(value, 0f, 1f);
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


