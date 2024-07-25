using Unity.Entities;

namespace ECSExperiment.Wheels
{
    public struct WheelTag : IComponentData
    {
        public bool IsGrounded;
    }
}
