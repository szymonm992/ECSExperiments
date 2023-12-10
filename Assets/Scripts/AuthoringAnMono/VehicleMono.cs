using Unity.Entities;
using UnityEngine;

public class VehicleMono : MonoBehaviour
{
    public float VehicleMaximumForwardSpeed => vehicleMaximumForwardSpeed;
    public float VehicleMaximumBackwardSpeed => vehicleMaximumBackwardSpeed;

    [SerializeField] private float vehicleMaximumForwardSpeed;
    [SerializeField] private float vehicleMaximumBackwardSpeed;

    public class Baker : Baker<VehicleMono>
    {
        public override void Bake(VehicleMono authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new VehicleEntityProperties
            {
                VehicleMaximumForwardSpeed = authoring.VehicleMaximumForwardSpeed,
                VehicleMaximumBackwardSpeed = authoring.VehicleMaximumBackwardSpeed,
            });

            AddComponent(entity, new EntityInputsData { });
        }
    }

}
