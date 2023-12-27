using UnityEngine;
using Unity.Entities;

public class WheelMono : MonoBehaviour
{
    public bool IsGrounded { get; set; }
    public float Spring => spring;
    public float SpringStiffness => springStiffness;
    public float Damper => damper;
    public float DamperStiffness => damperStiffness;
    public float Mass => mass;
    public float Radius => radius;
    public float Thickness => thickness;
    public float SpringLength => springLength;
    public WheelSide WheelSide => wheelSide;
    public bool CanDrive => canDrive;

    [SerializeField] private float spring;
    [SerializeField] private float damper;
    [SerializeField] private float mass;
    [SerializeField] private float radius;
    [SerializeField] private float thickness;
    [SerializeField] private float springLength;
    [SerializeField] private float springStiffness;
    [SerializeField] private float damperStiffness;
    [SerializeField] private WheelSide wheelSide;
    [SerializeField] private bool canDrive;

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

                var wheelProperties = new WheelProperties
                {
                    Entity = wheelEntity,
                    Spring = wheelAuthoring.Spring,
                    Damper = wheelAuthoring.Damper,
                    Mass = wheelAuthoring.Mass,
                    Radius = wheelAuthoring.Radius,
                    Thickness = wheelAuthoring.Thickness,
                    SpringLength = wheelAuthoring.SpringLength,
                    Side = wheelAuthoring.WheelSide,
                    CanDrive = wheelAuthoring.CanDrive,
                };

                EntityManager.AddComponentData(wheelEntity, wheelProperties);
                EntityManager.AddComponent<WheelHitData>(wheelEntity);

            }).Run();
    }
}

public enum WheelSide
{
    Center = 0,
    Left = -1,
    Right = 1,
}

