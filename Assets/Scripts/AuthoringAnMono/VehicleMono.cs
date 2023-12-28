using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class VehicleMono : MonoBehaviour
{
    public IEnumerable<GameObject> Wheels => wheels;
    public float VehicleMaximumForwardSpeed => vehicleMaximumForwardSpeed;
    public float VehicleMaximumBackwardSpeed => vehicleMaximumBackwardSpeed;

    [SerializeField] private float vehicleMaximumForwardSpeed;
    [SerializeField] private float vehicleMaximumBackwardSpeed;
    [SerializeField] private GameObject[] wheels;

    public class Baker : Baker<VehicleMono>
    {
        public override void Bake(VehicleMono authoring)
        {
            var entity = GetEntity(authoring.gameObject, TransformUsageFlags.Dynamic);
            Debug.Assert(authoring.Wheels != null && authoring.Wheels.Any(), "Wheels list cannot be empty");

          
            AddComponent(entity, new InputsData { });

            var vehicleBakingData = new VehicleBakingData()
            {
                Authoring = authoring,
                Wheels = GetWheelInfo(authoring.Wheels, Allocator.TempJob),
            };

            AddComponent(entity, vehicleBakingData);
        }

        NativeArray<Entity> GetWheelInfo(IEnumerable<GameObject> wheels, Allocator allocator)
        {
            if (wheels == null)
            {
                return default;
            }

            var array = new NativeArray<Entity>(wheels.Count(), allocator);
            int i = 0;

            foreach (var wheel in wheels)
            {
                array[i++] = GetEntity(wheel, TransformUsageFlags.Dynamic);
            }

            return array;
        }
    }
}



[TemporaryBakingType]
public struct VehicleBakingData : IComponentData
{
    public UnityObjectRef<VehicleMono> Authoring;
    public NativeArray<Entity> Wheels;
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

                var vehicleProperties = new VehicleProperties
                {
                    VehicleMaximumBackwardSpeed = vehicleAuthoring.VehicleMaximumBackwardSpeed,
                    VehicleMaximumForwardSpeed = vehicleAuthoring.VehicleMaximumForwardSpeed,
                    VehicleEntity = entity,
                    WheelsAmount = vehicleAuthoring.Wheels.Count(),
                };
                /*
                foreach (var wheelEntity in vehicleBakingData.Wheels)
                {
                    var wheelBakingData = EntityManager.GetComponentData<WheelBakingData>(wheelEntity);
                    wheelBakingData.VehicleEntity = entity;
                    EntityManager.SetComponentData(wheelEntity, wheelBakingData);
                }
                */
                EntityManager.AddComponentData(entity, vehicleProperties);

            }).Run();
    }
}
