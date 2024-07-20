using Unity.Burst;
using Unity.Entities;

namespace ECSExperiment.Wheels
{
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct UpdateUISystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<VehicleProperties>();
        }

        public void OnUpdate(ref SystemState state)
        {
            state.Enabled = false;

            if (HUDController.Instance == null)
            {
                return;
            }

            HUDController.Instance.InitializeHUD();
        }
    }
}
