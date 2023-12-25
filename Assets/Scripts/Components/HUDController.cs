using UnityEngine;
using TMPro;
using Unity.Entities;
using System.Collections;

public class HUDController : MonoBehaviour
{
    private const float EXECUTION_DELAY = 0.3f;

    [SerializeField] private TextMeshProUGUI speedometer;

    private EntityManager entityManager;
    private Entity playerEntity;
    private bool initialized = false;

    private IEnumerator Start()
    {
        initialized = false;
        entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;

        yield return new WaitForSeconds(EXECUTION_DELAY);

        //We can replace it with PlayerTag or another marker
        playerEntity = entityManager.CreateEntityQuery(typeof(EntityInputsData)).GetSingletonEntity();
        initialized = true;
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
