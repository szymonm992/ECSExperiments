using UnityEngine;
using Unity.Entities;
using UnityEditor.PackageManager;

public class WheelMono : MonoBehaviour
{
    public float Spring => spring;
    public float SpringStiffness => springStiffness;
    public float Damper => damper;
    public float DamperStiffness => damperStiffness;
    public float Mass => mass;
    public float Radius => radius;
    public float Thickness => thickness;
    public float RestLength => restLength;

    [SerializeField] private float spring;
    [SerializeField] private float damper;
    [SerializeField] private float mass;
    [SerializeField] private float radius;
    [SerializeField] private float thickness;
    [SerializeField] private float restLength;
    [SerializeField] private float springStiffness;
    [SerializeField] private float damperStiffness;

    public class Baker : Baker<WheelMono>
    {
        public override void Bake(WheelMono authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            var wheelBakingData = new WheelBakingData()
            {
                Authoring = authoring,
                WheelEntity = entity,
            };

            AddComponent(entity, wheelBakingData);
        }
    }
}

[TemporaryBakingType]
public struct WheelBakingData : IComponentData
{
    public UnityObjectRef<WheelMono> Authoring;
    public Entity WheelEntity;
}


[WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
[UpdateInGroup(typeof(PostBakingSystemGroup))]
public partial class WheelBaker : SystemBase
{
    protected override void OnUpdate()
    {
        Entities
            .WithEntityQueryOptions(EntityQueryOptions.IncludePrefab)
            .WithStructuralChanges()
            .ForEach((Entity entity, ref WheelBakingData wheelBakingData) =>
            {
                var wheelAuthoring = wheelBakingData.Authoring.Value;
                var wheelEntity = wheelBakingData.WheelEntity;

                var suspension = new Suspension
                {
                    RestLength = wheelAuthoring.RestLength,
                    SpringStiffness = wheelAuthoring.SpringStiffness,
                    DamperStiffness = wheelAuthoring.DamperStiffness,
                    SpringLength = wheelAuthoring.RestLength
                };

                var wheelProperties = new WheelProperties
                {
                    Entity = wheelEntity,
                    Spring = wheelAuthoring.Spring,
                    Damper = wheelAuthoring.Damper,
                    Mass = wheelAuthoring.Mass,
                    Radius = wheelAuthoring.Radius,
                    Thickness = wheelAuthoring.Thickness,
                    Travel = wheelAuthoring.RestLength,
                };

                EntityManager.AddComponentData(wheelEntity, wheelProperties);
                EntityManager.AddComponentData(wheelEntity, suspension);
                EntityManager.AddComponent<WheelHitData>(wheelEntity);

            }).Run();
    }
}

