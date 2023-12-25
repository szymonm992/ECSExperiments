using UnityEngine;
using TMPro;
using Unity.Entities;

public class HUDController : MonoBehaviour
{
    public static HUDController Instance;

    [SerializeField] private TextMeshProUGUI speedometer;

    private EntityManager entityManager;
    private Entity playerEntity;
    private bool initialized = false;

    public void InitializeHUD()
    {
        initialized = false;
        entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
       
        //We can replace it with PlayerTag or another marker
        playerEntity = entityManager.CreateEntityQuery(typeof(EntityInputsData)).GetSingletonEntity();
        initialized = true;

        Debug.Log("Initialized HUD Controller");
    }

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
        }
    }

    private void Update()
    {
        if (initialized)
        {
            var currentSpeed = entityManager.GetComponentData<VehicleEntityProperties>(playerEntity).CurrentSpeed;
            speedometer.text = currentSpeed.ToString("F1");
        }
    }
}
