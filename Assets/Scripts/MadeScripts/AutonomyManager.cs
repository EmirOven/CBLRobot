using UnityEngine;
using UnityEngine.UI;

public class AutonomyManager : MonoBehaviour
{
    public bool IsAutonomous = false;
    public Text statusText;

    void Start()
    {
        
    }

    void Update()
    {
        
        if (!IsAutonomous && Input.GetKeyDown(KeyCode.N))
        {
            ToggleAutonomy();
        }
    }

    public void ToggleAutonomy()
    {
        IsAutonomous = !IsAutonomous;
        if (statusText != null)
            statusText.text = IsAutonomous ? "AUTONOMOUS" : "MANUAL";
    }
}
