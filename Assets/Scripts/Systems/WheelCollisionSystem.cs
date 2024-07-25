using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Physics.Systems;

namespace ECSExperiment.Wheels
{
    [BurstCompile]
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsSystemGroup))]
    public partial struct TireCollisionSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.TempJob);

            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            var tireTagHandle = SystemAPI.GetComponentLookup<TireProperties>(false);
            var colliderTagHandle = SystemAPI.GetComponentLookup<PhysicsCollider>(true);

            var collisionJob = new TireCollisionEventJob
            {
                TireTagLookup = tireTagHandle,
                ColliderTagLookup = colliderTagHandle,  
                PhysicsWorld = physicsWorld,
                ECB = ecb.AsParallelWriter()
            };

            state.Dependency = collisionJob.Schedule(SystemAPI.GetSingleton<SimulationSingleton>(), state.Dependency);
            state.Dependency.Complete();
            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        [BurstCompile]
        struct TireCollisionEventJob : ICollisionEventsJob
        {
            [ReadOnly] public ComponentLookup<PhysicsCollider> ColliderTagLookup;
            [ReadOnly] public PhysicsWorld PhysicsWorld;

            public ComponentLookup<TireProperties> TireTagLookup;
            public EntityCommandBuffer.ParallelWriter ECB;

            public void Execute(CollisionEvent collisionEvent)
            {
                var firstEntity = collisionEvent.EntityA;
                var secondEntity = collisionEvent.EntityB;

                bool isFirstEntityTire = TireTagLookup.HasComponent(firstEntity);
                bool isSecondEntityTire = TireTagLookup.HasComponent(secondEntity);
                bool isFirstEntityCollideable = ColliderTagLookup.HasComponent(firstEntity);
                bool isSecondEntityCollideable = ColliderTagLookup.HasComponent(secondEntity);

                var tireEntity = isFirstEntityTire ? firstEntity : secondEntity;
                var tireProperties = TireTagLookup[tireEntity];

                bool isTireGrounded = isFirstEntityTire && isSecondEntityCollideable || isSecondEntityTire && isFirstEntityCollideable;

                if (isTireGrounded)
                {
                    var collisionDetails = collisionEvent.CalculateDetails(ref PhysicsWorld);
                    var parallelVector = math.normalize(math.cross(collisionEvent.Normal, math.right()));

                    UpdateTireProperties(ref tireProperties, true, collisionDetails.EstimatedContactPointPositions, collisionEvent.Normal, parallelVector);

                    Debug.DrawRay(collisionDetails.EstimatedContactPointPositions[0], parallelVector, Color.yellow);
                }
                else
                {
                    UpdateTireProperties(ref tireProperties, false, default, math.up(), math.forward());
                }

                TireTagLookup[tireEntity] = tireProperties;
            }

            private void UpdateTireProperties(ref TireProperties tireProperties, bool hasContact, NativeArray<float3> contactPoints, float3 contactNormal, float3 contactSurfaceVector)
            {
                tireProperties.SurfaceNormal = contactNormal;
                tireProperties.IsGrounded = hasContact;
                tireProperties.SurfaceVector = contactSurfaceVector;
            }
        }
    }
}
