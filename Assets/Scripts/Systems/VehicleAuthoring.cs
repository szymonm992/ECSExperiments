using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;


public class VehicleAuthoring : MonoBehaviour
{
    public bool ActiveAtStart;

    [Header("Handling")] public float TopSpeed = 10.0f;
    public float MaxSteeringAngle = 30.0f;
    [Range(0f, 1f)] public float SteeringDamping = 0.1f;
    [Range(0f, 1f)] public float SpeedDamping = 0.01f;


    void OnValidate()
    {
        TopSpeed = math.max(0f, TopSpeed);
        MaxSteeringAngle = math.max(0f, MaxSteeringAngle);
        SteeringDamping = math.clamp(SteeringDamping, 0f, 1f);
        SpeedDamping = math.clamp(SpeedDamping, 0f, 1f);
    }

    class VehicleBaker : Baker<VehicleAuthoring>
    {
        public override void Bake(VehicleAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent<Vehicle>(entity);

            AddComponent(entity, new VehicleSpeed
            {
                TopSpeed = authoring.TopSpeed,
                Damping = authoring.SpeedDamping
            });

            AddComponent(entity, new VehicleSteering
            {
                MaxSteeringAngle = math.radians(authoring.MaxSteeringAngle),
                Damping = authoring.SteeringDamping
            });
        }
    }
}

struct Vehicle : IComponentData
{
}

struct VehicleSpeed : IComponentData
{
    public float TopSpeed;
    public float DesiredSpeed;
    public float Damping;
    public byte DriveEngaged;
}

struct VehicleSteering : IComponentData
{
    public float MaxSteeringAngle;
    public float DesiredSteeringAngle;
    public float Damping;
}

    

