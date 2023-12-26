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

            var vehicleProperties = new VehicleEntityProperties
            {
                VehicleMaximumBackwardSpeed = authoring.VehicleMaximumBackwardSpeed,
                VehicleMaximumForwardSpeed = authoring.VehicleMaximumForwardSpeed,
                VehicleEntity = entity,
            };

            AddComponent(entity, vehicleProperties);
            AddComponent(entity, new EntityInputsData { });
        }
    }
}


[TemporaryBakingType]
public struct VehicleBakingData : IComponentData
{
    public UnityObjectRef<VehicleMono> Authoring;
}



[WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
[UpdateInGroup(typeof(PostBakingSystemGroup))]
public partial class VehicleBaker : SystemBase
{
    protected override void OnUpdate()
    {
        Entities
            .WithEntityQueryOptions(EntityQueryOptions.IncludePrefab)
            .WithStructuralChanges()
            .ForEach((Entity entity, ref VehicleBakingData vehicleBakingData) =>
            {
                var vehicleAuthoring = vehicleBakingData.Authoring.Value;
            }).Run();

    }
}
