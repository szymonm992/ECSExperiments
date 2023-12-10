using Unity.Burst;
using Unity.Entities;


    [RequireMatchingQueriesForUpdate]
    [UpdateAfter(typeof(ChangeActiveVehicleSystem))]
    partial struct CameraFollowSystem : ISystem
    {
      
    }

