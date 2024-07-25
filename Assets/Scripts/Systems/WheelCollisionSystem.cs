using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using UnityEngine;
using Unity.Collections;

namespace ECSExperiment.Wheels
{
    [BurstCompile]
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public partial struct WheelCollisionSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.TempJob);
            var collisionJob = new WheelCollisionEventJob
            {
                ECB = ecb.AsParallelWriter()
            };

            state.Dependency = collisionJob.Schedule(SystemAPI.GetSingleton<SimulationSingleton>(), state.Dependency);
            state.Dependency.Complete();
            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        private struct WheelCollisionEventJob : ICollisionEventsJob
        {
            public EntityCommandBuffer.ParallelWriter ECB;

            public void Execute(CollisionEvent collisionEvent)
            {
                var firstEntity = collisionEvent.EntityA;
                var secondEntity = collisionEvent.EntityB;
            }
        }
    }
}
