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

namespace ECSExperiment.Wheels
{
    [UpdateInGroup(typeof(PhysicsSimulationGroup), OrderFirst = true)]
    public partial struct WheelRaycastSystem : ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency.Complete();
            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            List<(int rigId, float3 force, float3 point)> impulsePoints = new();

            foreach (var (wheelProperties, wheelHitData, wheelCastOriginLocalTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRW<WheelHitData>, RefRO<LocalTransform>>())
            {
                var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);

                if (-1 == rigidbodyIndex || rigidbodyIndex >= physicsWorld.NumDynamicBodies)
                {
                    return;
                }

                var vehicleRigidbodyTransform = LocalTransform.FromMatrix(float4x4.TRS(
                        PhysicsWorldExtensions.GetPosition(physicsWorld, rigidbodyIndex),
                        PhysicsWorldExtensions.GetRotation(physicsWorld, rigidbodyIndex),
                        new float3(1f, 1f, 1f)
                    ));

                var wheelCastOriginGlobalTransform = vehicleRigidbodyTransform.TransformTransform(wheelCastOriginLocalTransform.ValueRO);
                var springDirection = math.mul(wheelCastOriginGlobalTransform.Rotation, math.up());
                var wheelRightLocal = math.mul(wheelCastOriginGlobalTransform.Rotation, math.right());

                var rayStart = wheelCastOriginGlobalTransform.Position + (springDirection * wheelProperties.ValueRO.Radius);
                var rayEnd = rayStart - springDirection * (wheelProperties.ValueRO.SpringLength + wheelProperties.ValueRO.Radius);

                var raycastInput = new RaycastInput
                {
                    Start = rayStart,
                    End = rayEnd,
                    Filter = CollisionFilter.Default,
                };

                wheelHitData.ValueRW.WheelCenter = rayStart - (springDirection * wheelProperties.ValueRO.SpringLength);
                wheelHitData.ValueRW.Velocity = physicsWorld.GetLinearVelocity(rigidbodyIndex, wheelHitData.ValueRO.WheelCenter);
                wheelHitData.ValueRW.HasHit = false;

                Color rayColor = wheelProperties.ValueRO.IsGrounded ? Color.green : Color.red;
                Debug.DrawRay(rayStart, rayEnd - rayStart, rayColor);

                if (physicsWorld.CollisionWorld.CastRay(raycastInput, out var result))
                {
                    float raycastDistance = math.distance(result.Position, rayStart);

                    wheelProperties.ValueRW.IsGrounded = true;
                    wheelHitData.ValueRW.HasHit = true;
                    wheelHitData.ValueRW.HitPoint = result.Position;
                    wheelHitData.ValueRW.SurfaceFriction = result.Material.Friction;
                    wheelHitData.ValueRW.Distance = (raycastDistance - wheelProperties.ValueRO.Radius);
                    wheelHitData.ValueRW.WheelCenter = rayStart - (springDirection * raycastDistance);


                    var velocityAtWheel = physicsWorld.GetLinearVelocity(rigidbodyIndex, wheelHitData.ValueRO.WheelCenter);
                    var invertedWheelsCount = (1f / 4f);
                    var currentSpeedUp = math.dot(velocityAtWheel, springDirection);

                    float3 lvA = currentSpeedUp * springDirection;
                    float3 lvB = physicsWorld.GetLinearVelocity(result.RigidBodyIndex, wheelHitData.ValueRO.WheelCenter);
                    float3 totalSuspensionForce = (wheelProperties.ValueRO.Spring * (result.Position - rayEnd))
                        + (wheelProperties.ValueRO.Damper * (lvB - lvA)) * invertedWheelsCount;

                    Debug.DrawRay(wheelHitData.ValueRO.WheelCenter, math.up(), Color.yellow);
                    Debug.DrawRay(wheelHitData.ValueRO.WheelCenter, springDirection, Color.cyan);

                    float impulseUp = math.dot(totalSuspensionForce, springDirection);
                    float downForceLimit = -0.25f;

                    if (downForceLimit < impulseUp)
                    {
                        totalSuspensionForce = impulseUp * springDirection;
                        //physicsWorld.ApplyImpulse(rigidbodyIndex, totalSuspensionForce, wheelHitData.ValueRO.WheelCenter);
                        impulsePoints.Add(new(rigidbodyIndex, totalSuspensionForce, wheelHitData.ValueRO.WheelCenter));
                    }
                }
                else
                {
                    wheelProperties.ValueRW.IsGrounded = false;
                }

#if UNITY_EDITOR
                Color color = wheelProperties.ValueRO.IsGrounded ? Color.green : Color.red;
                float3 localRightDirection = math.mul(wheelCastOriginGlobalTransform.Rotation,
                    new float3((int)wheelProperties.ValueRO.Side, 0, 0));
                Debug.DrawRay(wheelHitData.ValueRO.WheelCenter, localRightDirection, color);
#endif
            }


            foreach (var chuj in impulsePoints)
            {
                physicsWorld.ApplyImpulse(chuj.rigId, chuj.force, chuj.point);
            }
        }
    }
}


