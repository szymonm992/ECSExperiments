using Unity.Burst;
using Unity.Entities;

[BurstCompile]
[UpdateInGroup(typeof(InitializationSystemGroup))]
public partial struct UpdateUISystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        //We do not want it to run until we have entity setup
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
