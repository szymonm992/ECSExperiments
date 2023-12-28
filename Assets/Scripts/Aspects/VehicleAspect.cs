using Unity.Entities;

namespace ECSExperiment
{
    public readonly partial struct VehicleAspect : IAspect
    {
        readonly RefRW<VehicleProperties> vehicleProperties;
        readonly RefRW<InputsData> inputsData;

        public VehicleProperties VehicleProperties => vehicleProperties.ValueRO;
        public InputsData InputsData => inputsData.ValueRO;
    }
}
