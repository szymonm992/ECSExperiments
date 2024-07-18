using Unity.Entities;
using Unity.Mathematics;

namespace ECSExperiment.Wheels
{
    [InternalBufferCapacity(50)]
    public struct ForceAccumulationBufferElement : IBufferElementData
    {
        public float3 point; 
        public float3 force; 
    }
}
