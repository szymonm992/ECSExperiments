using Unity.Entities;
using Unity.Mathematics;

namespace ECSExperiment.Wheels
{
    public struct TireProperties : IComponentData
    {
        public float3 SurfaceNormal;
        public float3 SurfaceVector;
        public bool IsGrounded;
    }
}
