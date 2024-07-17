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

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    public partial struct WheelRaycastSystem : ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency.Complete();

            var physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
            var commandBuffer = new EntityCommandBuffer(Allocator.TempJob);

            foreach (var (wheelProperties, wheelHitData, wheelCastOriginGlobalTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRW<WheelHitData>, RefRO<LocalToWorld>>())
            {
                var springDirection = math.mul(wheelCastOriginGlobalTransform.ValueRO.Rotation, math.up());
                var wheelRightLocal = math.mul(wheelCastOriginGlobalTransform.ValueRO.Rotation, math.right());
                int vehicleRigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);

                float3 localWheelPositionRelativeToMotherfuckingRigidkurwaBody = SystemAPI.GetComponent<LocalTransform>(wheelProperties.ValueRO.Entity).Position;

                if (-1 == vehicleRigidbodyIndex || vehicleRigidbodyIndex >= physicsWorld.NumDynamicBodies)
                {
                    return;
                }

                float springMaxLength = 1f;
                float suspensionRestDistance = 0.5f;
                float springLength = springMaxLength;

                float3 wheelPosition = wheelCastOriginGlobalTransform.ValueRO.Position - (wheelProperties.ValueRO.Radius + (springDirection * suspensionRestDistance)); 
                float3 rayStart = wheelCastOriginGlobalTransform.ValueRO.Position;
                float3 rayEnd = rayStart  - (springDirection * springMaxLength);

                var raycastInput = new RaycastInput
                {
                    Start = rayStart,
                    End = rayEnd,
                    Filter = CollisionFilter.Default
                };

                if (physicsWorld.CollisionWorld.CastRay(raycastInput, out var rayResult))
                {
                    float hitDistance = math.distance(rayStart, rayResult.Position);
                    springLength = hitDistance;

                    float3 wheelWorldVelocity = physicsWorld.GetLinearVelocity(vehicleRigidbodyIndex, rayResult.Position);
                    wheelPosition = (rayResult.Position + (springDirection * wheelProperties.ValueRO.Radius));

                    float offset = suspensionRestDistance - hitDistance;
                    float velocity = math.dot(springDirection, wheelWorldVelocity);
                    float force = (offset * wheelProperties.ValueRO.Spring) - (velocity * wheelProperties.ValueRO.Damper);

                    physicsWorld.ApplyImpulse(vehicleRigidbodyIndex, springDirection * force, rayResult.Position);

                    wheelHitData.ValueRW.HasHit = true;
                    wheelHitData.ValueRW.HitPoint = rayResult.Position;
                    wheelHitData.ValueRW.Normal = rayResult.SurfaceNormal;
                    wheelHitData.ValueRW.Distance = hitDistance;
                }
                else
                {
                    springLength = springMaxLength;

                    wheelHitData.ValueRW.HasHit = false;
                    wheelHitData.ValueRW.HitPoint = float3.zero;
                    wheelHitData.ValueRW.Normal = float3.zero;
                    wheelHitData.ValueRW.Distance = 0f;
                }

                Debug.DrawRay(wheelPosition, -wheelRightLocal, Color.red);
                Debug.DrawRay(raycastInput.Start, -(springDirection * springLength), wheelHitData.ValueRW.HasHit ? Color.green : Color.red);
            }

            commandBuffer.Playback(state.EntityManager);
            commandBuffer.Dispose();
        }
    }
}



        /*
        [UpdateInGroup(typeof(PhysicsSimulationGroup))]
        public partial struct WheelRaycastSystem : ISystem
        {
            [BurstCompile]
            public void OnUpdate(ref SystemState state)
            {
                var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

                var wheelRaycastJob = new WheelRaycastJob
                {
                    PhysicsWorld = physicsWorld,
                    LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                    WorldTransformLookup = SystemAPI.GetComponentLookup<LocalToWorld>(true),
                };

                state.Dependency = wheelRaycastJob.Schedule(state.Dependency);
            }
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct WheelRaycastJob : IJobEntity
        {
            public PhysicsWorld PhysicsWorld;
            [ReadOnly] public ComponentLookup<LocalTransform> LocalTransformLookup;
            [ReadOnly] public ComponentLookup<LocalToWorld> WorldTransformLookup;

            private void Execute(ref WheelProperties wheelProperties, ref WheelHitData wheelHitData)
            {
                var wheelTransform = WorldTransformLookup[wheelProperties.Entity];
                var vehicleTransform = LocalTransformLookup[wheelProperties.VehicleEntity];

                float3 localUpDirection = math.mul(vehicleTransform.Rotation, math.up());

                var rigidbodyIndex = PhysicsWorld.GetRigidBodyIndex(wheelProperties.VehicleEntity);
                var rayStart = wheelTransform.Position + (localUpDirection * wheelProperties.Radius);
                var rayEnd = rayStart - localUpDirection * (wheelProperties.SpringLength + wheelProperties.Radius);
                var rayCollisionFilter = PhysicsWorld.GetCollisionFilter(rigidbodyIndex);

                var raycastInput = new RaycastInput
                {
                    Start = rayStart,
                    End = rayEnd,
                    Filter = rayCollisionFilter
                };

                wheelHitData.WheelCenter = rayStart - (localUpDirection * wheelProperties.SpringLength);
                wheelHitData.Velocity = PhysicsWorld.GetLinearVelocity(rigidbodyIndex, wheelHitData.WheelCenter);
                wheelHitData.HasHit = false;

                wheelProperties.IsGrounded = PhysicsWorld.CollisionWorld.CastRay(raycastInput, out var result);

#if UNITY_EDITOR
                Color rayColor = wheelProperties.IsGrounded ? Color.green : Color.red;
                Debug.DrawRay(rayStart, rayEnd - rayStart, rayColor);
#endif

                if (wheelProperties.IsGrounded)
                {
                    float raycastDistance = math.distance(result.Position, rayStart);

                    wheelHitData.HasHit = true;
                    wheelHitData.HitPoint = result.Position;
                    wheelHitData.SurfaceFriction = result.Material.Friction;
                    wheelHitData.Normal = result.SurfaceNormal;
                    wheelHitData.Distance = (raycastDistance - wheelProperties.Radius);
                    wheelHitData.WheelCenter = rayStart - (localUpDirection * raycastDistance);


                    var velocityAtWheel = PhysicsWorld.GetLinearVelocity(rigidbodyIndex, wheelHitData.WheelCenter);
                    var invertedWheelsCount = (1f / 4f);
                    var currentSpeedUp = math.dot(velocityAtWheel, localUpDirection);

                    float3 lvA = currentSpeedUp * localUpDirection;
                    float3 lvB = PhysicsWorld.GetLinearVelocity(result.RigidBodyIndex, result.Position);
                    float3 totalSuspensionForce = (wheelProperties.Spring * (result.Position - rayEnd))
                        + (wheelProperties.Damper * (lvB - lvA)) * invertedWheelsCount;

                    Debug.DrawRay(wheelHitData.WheelCenter, math.up(), Color.yellow);
                    Debug.DrawRay(wheelHitData.WheelCenter, localUpDirection, Color.cyan);

                    float impulseUp = math.dot(totalSuspensionForce, localUpDirection);
                    float downForceLimit = -0.25f;

                    if (downForceLimit < impulseUp)
                    {
                        totalSuspensionForce = impulseUp * localUpDirection;
                        PhysicsWorld.ApplyImpulse(rigidbodyIndex, totalSuspensionForce, wheelHitData.WheelCenter);
                    }
                }

#if UNITY_EDITOR
                Color color = wheelProperties.IsGrounded ? Color.green : Color.red;
                float3 localRightDirection = math.mul(wheelTransform.Rotation,
                    new float3((int)wheelProperties.Side, 0, 0));
                Debug.DrawRay(wheelHitData.WheelCenter, localRightDirection, color);
#endif
            }
        }
    }*/

