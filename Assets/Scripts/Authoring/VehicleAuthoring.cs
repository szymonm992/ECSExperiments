using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class VehicleAuthoring : MonoBehaviour
{
    public float MaxForwardSpeed => maxForwardSpeed;
    public float MaxBackwardSpeed => maxBackwardSpeed;

    [SerializeField] private float maxForwardSpeed;
    [SerializeField] private float maxBackwardSpeed;

    public class VehicleBaker : Baker<VehicleAuthoring>
    {
        public override void Bake(VehicleAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(entity, new VehicleSpeed
            {
                MaxForwardSpeed = authoring.MaxForwardSpeed,
                MaxBackwardSpeed = authoring.MaxBackwardSpeed,  
            });
        }
    }

    public struct VehicleSpeed : IComponentData
    {
        public float MaxForwardSpeed;
        public float MaxBackwardSpeed;
    }

    private void OnValidate()
    {
        maxForwardSpeed = math.max(0f, maxForwardSpeed);
        maxBackwardSpeed = math.max(0f, maxBackwardSpeed);
    }
}
