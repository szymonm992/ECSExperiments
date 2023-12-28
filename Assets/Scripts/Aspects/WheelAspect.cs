using Unity.Entities;

namespace ECSExperiment.Wheels
{
    public readonly partial struct WheelAspect : IAspect
    {
        readonly RefRW<WheelProperties> wheelProperties;
        readonly RefRW<WheelHitData> hitData;
        public WheelProperties WheelProperties => wheelProperties.ValueRO;
        public WheelHitData HitData => hitData.ValueRO;
    }
}
