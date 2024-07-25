using Unity.Entities;

namespace ECSExperiment.Wheels
{
    public struct TireProperties : IComponentData
    {
        public bool IsGrounded;
    }
}
