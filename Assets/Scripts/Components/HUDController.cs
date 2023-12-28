using UnityEngine;
using TMPro;
using Unity.Entities;
using DG.Tweening;

public class HUDController : MonoBehaviour
{
    public static HUDController Instance;

    [SerializeField] private CanvasGroup mainCanvasGroup;
    [SerializeField] private TextMeshProUGUI speedometer;

    private EntityManager entityManager;
    private Entity playerEntity;
    private bool initialized = false;
    private bool isDisplayed = true;

    public void InitializeHUD()
    {
        /*
        initialized = false;
        entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
        
        //We can replace it with PlayerTag or another marker
        playerEntity = entityManager.CreateEntityQuery(typeof(InputsData)).GetSingletonEntity();
        initialized = true;

        ToggleHUD(true, 2f);
        Debug.Log("Initialized HUD Controller");*/
    }

    public void ToggleHUD(bool value, float duration = 0f)
    {
        if (isDisplayed == value)
        {
            Debug.Log($"HUD toggle value is already at state {value}");
        }

        ToggleHUDInternal(value, duration); 
    }

    private void ToggleHUDInternal(bool value, float duration = 0f)
    {
        isDisplayed = value;
        float desiredAlphaValue = value ? 1f : 0f;

        if (duration > 0)
        {
            mainCanvasGroup.DOFade(desiredAlphaValue, duration);
        }
        else
        {
            mainCanvasGroup.alpha = desiredAlphaValue;
        }
    }

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
        }

        ToggleHUDInternal(false);
    }

    private void Update()
    {
        if (initialized)
        {
            var currentSpeed = entityManager.GetComponentData<VehicleProperties>(playerEntity).CurrentSpeed;
            speedometer.text = currentSpeed.ToString("F1");
        }
    }
}
