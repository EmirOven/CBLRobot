using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Keeps track of a 2D occupancy grid (centered on Unity's origin) and spawns
/// a sphere in each cell the first time it's hit. Attach this to an empty GameObject.
/// </summary>
public class GridManager : MonoBehaviour
{
    [Header("Grid Parameters")]
    [Tooltip("Number of cells along the X (width) axis.")]
    public int gridWidth = 200;       // e.g. 200 cells wide (centered on 0)
    [Tooltip("Number of cells along the Z (height) axis.")]
    public int gridHeight = 200;      // e.g. 200 cells tall (centered on 0)
    [Tooltip("Meters per cell (e.g. 0.1 = each cell is 0.1m × 0.1m).")]
    public float cellSize = 0.1f;     // 10 cm resolution

    [Header("Sphere Prefab")]
    [Tooltip("A prefab (small sphere) to instantiate when a new cell is hit.")]
    public GameObject spherePrefab;

    // Internal: tracks which grid (ix, iz) indices are already occupied
    private HashSet<Vector2Int> occupiedCells;

    private void Awake()
    {
        occupiedCells = new HashSet<Vector2Int>();

        if (spherePrefab == null)
        {
            Debug.LogError("GridManager: Please assign a spherePrefab in the Inspector.");
        }
    }

    /// <summary>
    /// Call this every time you have a new world-position (x, z) to mark.
    /// If its grid cell is empty, instantiates a sphere there (and marks it occupied).
    /// </summary>
    /// <param name="worldPos">Unity-world coordinates of the detected point (y is ignored).</param>
    public void PlaceSphereIfCellEmpty(Vector3 worldPos)
    {
        // 1) Compute grid indices (ix, iz) from worldPos.x, worldPos.z
        // The grid is centered on (0,0). So the bottom-left corner in X is at: -halfWidth,
        // top-right is +halfWidth. Same for Z.
        float halfWidth = (gridWidth * cellSize) / 2f;
        float halfHeight = (gridHeight * cellSize) / 2f;

        // ix = floor( (worldPos.x + halfWidth) / cellSize )
        // iz = floor( (worldPos.z + halfHeight) / cellSize )
        int ix = Mathf.FloorToInt((worldPos.x + halfWidth) / cellSize);
        int iz = Mathf.FloorToInt((worldPos.z + halfHeight) / cellSize);

        // If outside grid, ignore
        if (ix < 0 || ix >= gridWidth || iz < 0 || iz >= gridHeight)
        {
            return;
        }

        var cell = new Vector2Int(ix, iz);
        if (occupiedCells.Contains(cell))
        {
            // Already spawned a sphere here, do nothing
            return;
        }

        // Otherwise, spawn a sphere exactly at (worldPos.x, some_y, worldPos.z).
        // We choose y = 0.05f so it hovers slightly above the floor.
        Vector3 spawnPos = new Vector3(worldPos.x, 0.05f, worldPos.z);
        GameObject sph = Instantiate(spherePrefab, spawnPos, Quaternion.identity);
        sph.tag = "ObstacleSphere";

        occupiedCells.Add(cell);
    }

    /// <summary>
    /// (Optional) Clears all spheres and resets the grid occupancy.
    /// </summary>
    public void ClearAll()
    {
        occupiedCells.Clear();
        // Destroy all GameObjects tagged "ObstacleSphere"
        foreach (GameObject go in GameObject.FindGameObjectsWithTag("ObstacleSphere"))
        {
            Destroy(go);
        }
    }

    // (Optional) To visualize the grid in the Editor:
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        float halfW = (gridWidth * cellSize) / 2f;
        float halfH = (gridHeight * cellSize) / 2f;

        // Draw vertical lines
        for (int i = 0; i <= gridWidth; i++)
        {
            float x = -halfW + i * cellSize;
            Vector3 p1 = new Vector3(x, 0f, -halfH);
            Vector3 p2 = new Vector3(x, 0f, +halfH);
            Gizmos.DrawLine(p1, p2);
        }
        // Draw horizontal lines
        for (int j = 0; j <= gridHeight; j++)
        {
            float z = -halfH + j * cellSize;
            Vector3 p1 = new Vector3(-halfW, 0f, z);
            Vector3 p2 = new Vector3(+halfW, 0f, z);
            Gizmos.DrawLine(p1, p2);
        }
    }
}
