using UnityEngine;
using UnityEngine.UI;

public class AutonomyManager : MonoBehaviour
{
    public bool IsAutonomous = false;
    public Text statusText;

    private PatrolOverride patrolOverride;

    void Start()
    {
        // Get the PatrolOverride component from the same GameObject
        patrolOverride = GetComponent<PatrolOverride>();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.N))
        {
            Debug.Log("Key is Pressed");
            ToggleAutonomy();
        }
    }

    public void ToggleAutonomy()
    {
        IsAutonomous = !IsAutonomous;

        if (statusText != null)
            statusText.text = IsAutonomous ? "AUTONOMOUS" : "MANUAL";

        if (patrolOverride != null)
        {
            if (IsAutonomous)
                patrolOverride.ResumePatrol();  // Send False to ROS
            else
                patrolOverride.PausePatrol();   // Send True to ROS
        }
    }
}

