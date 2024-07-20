using UnityEngine;
using Unity.Entities;

namespace ECSExperiment.Wheels
{
    public class WheelAuthoring : MonoBehaviour
    {
        public bool IsGrounded { get; set; }
        public float Spring => spring;
        public float Damper => damper;
        public float Mass => mass;
        public float Radius => radius;
        public float SpringLength => springLength;
        public WheelSide WheelSide => wheelSide;
        public bool CanDrive => canDrive;
        public GameObject WheelVisualModel => wheelVisualModel;

        [SerializeField] private float spring;
        [SerializeField] private float damper;
        [SerializeField] private float mass;
        [SerializeField] private float radius;
        [SerializeField] private float springLength;
        [SerializeField] private WheelSide wheelSide;
        [SerializeField] private bool canDrive;
        [SerializeField] private GameObject wheelVisualModel;

        public class Baker : Baker<WheelAuthoring>
        {
            public override void Bake(WheelAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                var visualModelEntity = authoring.WheelVisualModel ? GetEntity(authoring.WheelVisualModel, TransformUsageFlags.Dynamic) : default;
              
                var wheelBakingData = new WheelBakingData()
                {
                    Authoring = authoring,
                    WheelEntity = entity,
                    WheelVisualModelEntity = visualModelEntity,
                };

                AddComponent(entity, wheelBakingData);
            }
        }
    }

    [TemporaryBakingType]
    public struct WheelBakingData : IComponentData
    {
        public UnityObjectRef<WheelAuthoring> Authoring;
        public Entity WheelEntity;
        public Entity WheelVisualModelEntity;
        public Entity VehicleEntity;
        public int WheelsAmount;
    }


    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    [UpdateInGroup(typeof(PostBakingSystemGroup))]
    [UpdateAfter(typeof(VehicleBaker))]
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
                    var wheelsAmountFraction = 1f / wheelBakingData.WheelsAmount;

                    var wheelProperties = new WheelProperties
                    {
                        Entity = wheelEntity,
                        VehicleEntity = wheelBakingData.VehicleEntity,
                        WheelVisualObjectEntity = wheelBakingData.WheelVisualModelEntity,

                        Spring = wheelAuthoring.Spring,
                        Damper = wheelAuthoring.Damper,
                        Mass = wheelAuthoring.Mass,
                        Radius = wheelAuthoring.Radius,
                        SpringLength = wheelAuthoring.SpringLength,
                        Side = wheelAuthoring.WheelSide,
                        CanDrive = wheelAuthoring.CanDrive,
                        WheelsAmountFraction = wheelsAmountFraction,
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
}

