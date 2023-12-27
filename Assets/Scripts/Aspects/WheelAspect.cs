using Unity.Entities;

public readonly partial struct WheelAspect : IAspect
{
    readonly RefRW<WheelProperties> wheelProperties;
    readonly RefRO<VehicleProperties> chassisReference;

    public Entity ChassisReferenceEntity => chassisReference.ValueRO.VehicleEntity;
    public WheelProperties WheelProperties => wheelProperties.ValueRO;
}
