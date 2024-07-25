using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using UnityEngine;
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
            var tireTagHandle = SystemAPI.GetComponentLookup<TireProperties>(false);
            var colliderTagHandle = SystemAPI.GetComponentLookup<PhysicsCollider>(true);

            var collisionJob = new TireCollisionEventJob
            {
                tireTagLookup = tireTagHandle,
                colliderTagLookup = colliderTagHandle,  
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
            [ReadOnly] public ComponentLookup<PhysicsCollider> colliderTagLookup;

            public ComponentLookup<TireProperties> tireTagLookup;
            public EntityCommandBuffer.ParallelWriter ECB;

            public void Execute(CollisionEvent collisionEvent)
            {
                var firstEntity = collisionEvent.EntityA;
                var secondEntity = collisionEvent.EntityB;

                bool isFirstEntityTire = tireTagLookup.HasComponent(firstEntity);
                bool isSecondEntityTire = tireTagLookup.HasComponent(secondEntity);
                bool isFirstEntityCollideable = colliderTagLookup.HasComponent(firstEntity);
                bool isSecondEntityCollideable = colliderTagLookup.HasComponent(secondEntity);

                var tireEntity = isFirstEntityTire ? firstEntity : secondEntity;
                var tireProperties = tireTagLookup[tireEntity];

                bool isTireGrounded = isFirstEntityTire && isSecondEntityCollideable || isSecondEntityTire && isFirstEntityCollideable;

                if (isTireGrounded)
                {
                    tireProperties.IsGrounded = true;
                }
                else
                {
                    tireProperties.IsGrounded = false;
                }

                tireTagLookup[tireEntity] = tireProperties;
            }
        }
    }
}
