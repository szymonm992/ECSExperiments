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
    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateBefore(typeof(PlayerMovementSystem))]
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

            float3 worldUpDirection = new float3(0, 1, 0);
            float3 localUpDirection = math.mul(vehicleTransform.Rotation, worldUpDirection);

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
                wheelHitData.Distance = (raycastDistance - wheelProperties.Radius);
                wheelHitData.WheelCenter = rayStart - (localUpDirection * raycastDistance);

#if UNITY_EDITOR
                Color color = wheelProperties.IsGrounded ? Color.green : Color.red;
                float3 localRightDirection = math.mul(wheelTransform.Rotation,
                    new float3((int)wheelProperties.Side, 0, 0));
                Debug.DrawRay(wheelHitData.WheelCenter, localRightDirection, color);
#endif

                var velocityAtWheel = PhysicsWorld.GetLinearVelocity(rigidbodyIndex, wheelHitData.WheelCenter);
                var invertedWheelsCount = (1f / 4f);
                var currentSpeedUp = math.dot(velocityAtWheel, localUpDirection);

                float3 lvA = currentSpeedUp * localUpDirection;
                float3 lvB = PhysicsWorld.GetLinearVelocity(result.RigidBodyIndex, result.Position);
                float3 totalSuspensionForce = (wheelProperties.Spring * (result.Position - rayEnd))
                    + (wheelProperties.Damper * (lvB - lvA)) * invertedWheelsCount;

                Debug.DrawRay(wheelTransform.Position, totalSuspensionForce, Color.black);

                float impulseUp = math.dot(totalSuspensionForce, localUpDirection);
                float downForceLimit = -0.25f;

                if (downForceLimit < impulseUp)
                {
                    totalSuspensionForce = impulseUp * localUpDirection;
                    PhysicsWorld.ApplyImpulse(rigidbodyIndex, totalSuspensionForce, wheelTransform.Position);
                }
            }
        }
    }
}

