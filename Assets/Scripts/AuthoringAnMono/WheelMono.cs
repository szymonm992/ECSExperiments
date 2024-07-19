using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public class WheelMono : MonoBehaviour
{
    public bool IsGrounded { get; set; }
    public float Spring => spring;
    public float Damper => damper;
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
    [SerializeField] private WheelSide wheelSide;
    [SerializeField] private bool canDrive;

    public float inertia = 2.2f;
    public float grip = 1.0f;
    public float brakeFrictionTorque = 4000;
    public float handbrakeFrictionTorque = 0;
    public float frictionTorque = 10;
    public float maxSteeringAngle = 28f;
    public float driveTorque = 0;
    public float driveFrictionTorque = 0;
    public float brake = 0;
    public float handbrake = 0;
    public float steering = 0;
    public float drivetrainInertia = 0;
    public float suspensionForceInput = 0;
    public float angularVelocity;
    public float slipRatio;
    public float slipVelo;
    public float compression;
    public float fullCompressionSpringForce;
    public Vector3 wheelVelo;
    public Vector3 localVelo;
    public Vector3 groundNormal;
    public float rotation;
    public float normalForce;
    public float slipAngle;
    public float maxSlip;
    public float maxAngle;
    public float oldAngle;

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
    public Entity VehicleEntity;
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

                var wheelProperties = new WheelProperties
                {
                    Entity = wheelEntity,
                    VehicleEntity = wheelBakingData.VehicleEntity,
                    Spring = wheelAuthoring.Spring,
                    Damper = wheelAuthoring.Damper,
                    Mass = wheelAuthoring.Mass,
                    Radius = wheelAuthoring.Radius,
                    Thickness = wheelAuthoring.Thickness,
                    SpringLength = wheelAuthoring.SpringLength,
                    Side = wheelAuthoring.WheelSide,
                    CanDrive = wheelAuthoring.CanDrive,

                    slipAngle = wheelAuthoring.slipAngle,
                    slipRatio = wheelAuthoring.slipRatio,
                    slipVelo = wheelAuthoring.slipVelo,
                    steering = wheelAuthoring.steering,
                    driveFrictionTorque = wheelAuthoring.driveFrictionTorque,
                    DriveTorque = wheelAuthoring.driveTorque,
                    drivetrainInertia = wheelAuthoring.drivetrainInertia,
                    angularVelocity = wheelAuthoring.angularVelocity,
                    brake = wheelAuthoring.brake,
                    Compression = wheelAuthoring.compression,
                    fullCompressionSpringForce = wheelAuthoring.fullCompressionSpringForce,
                    brakeFrictionTorque = wheelAuthoring.brakeFrictionTorque,
                    frictionTorque = wheelAuthoring.frictionTorque,
                    handbrakeFrictionTorque = wheelAuthoring.handbrakeFrictionTorque,
                    maxSlip = wheelAuthoring.maxSlip,
                    maxSteeringAngle = wheelAuthoring.maxSteeringAngle,
                    grip = wheelAuthoring.grip,
                    handbrake = wheelAuthoring.handbrake,
                    Inertia = wheelAuthoring.inertia,
                    LocalVelocity = wheelAuthoring.localVelo,
                    maxAngle = wheelAuthoring.maxAngle,
                    WheelVelocity = wheelAuthoring.wheelVelo,
                    oldAngle = wheelAuthoring.oldAngle,
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

