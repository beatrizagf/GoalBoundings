using Assets.Scripts.IAJ.Unity.Pathfinding;
using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using Assets.Scripts.IAJ.Unity.Pathfinding.Path;
using Assets.Scripts.IAJ.Unity.Pathfinding.GoalBounding;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding;

using UnityEngine;
using RAIN.Navigation;
using RAIN.Navigation.NavMesh;
using RAIN.Navigation.Graph;
using UnityEditor;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;

public class PathfindingManager : MonoBehaviour
{

    //public fields to be set in Unity Editor
    public GameObject endDebugSphere;
    public GameObject startDebugSphere;
    public Camera camera;
    public GameObject p1;
    public GameObject p2;
    public GameObject p3;
    public GameObject p4;
    public GameObject p5;
    public GameObject p6;

    //private fields for internal use only
    private Vector3 startPosition;
    private Vector3 endPosition;
    private NavMeshPathGraph navMesh;
    private int currentClickNumber;
    private GoalBoundingTable goalBoundsTable;

    private GlobalPath currentSolution;
    private bool draw;

    private bool finished;
    private GlobalPath smoothedPath;

    //public properties
    public AStarPathfinding AStarPathFinding { get; private set; }

    public void Initialize(NavMeshPathGraph navMeshGraph, AStarPathfinding pathfindingAlgorithm)
    {
        this.draw = false;
        this.navMesh = navMeshGraph;

        this.AStarPathFinding = pathfindingAlgorithm;
        this.AStarPathFinding.NodesPerFrame = 200;

        this.smoothedPath = new GlobalPath();
    }

    // Use this for initialization
    void Awake()
    {
        this.currentClickNumber = 1;

        //comentar o metodo que nao queremos usar e descomentar o que queremos usar (organizado do pior ao melhor):

        //AStar com Unordered List
        //this.Initialize(NavigationManager.Instance.NavMeshGraphs[0], new AStarPathfinding(NavigationManager.Instance.NavMeshGraphs[0], new SimpleUnorderedNodeList(), new HashtableSet(), new EuclidianDistanceHeuristic()));
        //AStar com PriorityHeap
        //this.Initialize(NavigationManager.Instance.NavMeshGraphs[0], new AStarPathfinding(NavigationManager.Instance.NavMeshGraphs[0], new NodePriorityHeap(), new HashtableSet(), new EuclidianDistanceHeuristic()));
        //NodeAStar
        //this.Initialize(NavigationManager.Instance.NavMeshGraphs[0], new NodeArrayAStarPathFinding(NavigationManager.Instance.NavMeshGraphs[0], new EuclidianDistanceHeuristic()));
        //GoalBounding
        this.goalBoundsTable = Resources.Load<GoalBoundingTable>("GoalBoundingTable");
        this.Initialize(NavigationManager.Instance.NavMeshGraphs[0], new GoalBoundingPathfinding(NavigationManager.Instance.NavMeshGraphs[0], new EuclidianDistanceHeuristic(), this.goalBoundsTable));

    }

    // Update is called once per frame
    void Update()
    {
        Vector3 position;

        if (Input.GetMouseButtonDown(0))
        {
            //if there is a valid position
            if (this.MouseClickPosition(out position))
            {
                //if this is the first click we're setting the start point
                if (this.currentClickNumber == 1)
                {
                    //show the start sphere, hide the end one
                    //this is just a small adjustment to better see the debug sphere
                    this.startDebugSphere.transform.position = position + Vector3.up;
                    this.startDebugSphere.SetActive(true);
                    this.endDebugSphere.SetActive(false);
                    this.currentClickNumber = 2;
                    this.startPosition = position;
                    this.currentSolution = null;
                    this.draw = false;
                }
                else
                {
                    //we're setting the end point
                    //this is just a small adjustment to better see the debug sphere
                    this.endDebugSphere.transform.position = position + Vector3.up;
                    this.endDebugSphere.SetActive(true);
                    this.currentClickNumber = 1;
                    this.endPosition = position;
                    this.draw = true;
                    //initialize the search algorithm
                    this.AStarPathFinding.InitializePathfindingSearch(this.startPosition, this.endPosition);
                }
            }
        }
        else if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            this.InitializePathFinding(this.p5.transform.localPosition, this.p6.transform.localPosition);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            this.InitializePathFinding(this.p1.transform.localPosition, this.p2.transform.localPosition);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            this.InitializePathFinding(this.p2.transform.localPosition, this.p4.transform.localPosition);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            this.InitializePathFinding(this.p2.transform.localPosition, this.p5.transform.localPosition);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha5))
        {
            this.InitializePathFinding(this.p1.transform.localPosition, this.p3.transform.localPosition);
        }

        //call the pathfinding method if the user specified a new goal
        if (this.AStarPathFinding.InProgress)
        {
            this.finished = this.AStarPathFinding.Search(out this.currentSolution);
        }

        if (finished)
        {
            smoothedPath.PathPositions.Clear();
            SmoothPath();
            finished = false;
        }


    }

    public void SmoothPath()
    {
        int i = 0;
        var positions = currentSolution.PathPositions;
        //smoothedPath.PathNodes.Add(currentSolution.PathNodes[0]);
        smoothedPath.PathPositions.Add(positions[0]);

        while (i+1 < positions.Count)
        {
            int j = i + 1;

            while (j < positions.Count)
            {
                if (j + 1 < positions.Count)
                {
                    var pos_start = positions[i];
                    var pos_next = positions[j + 1];
                    var direction = pos_next - pos_start;
                    float maxLookAhead = (pos_start - pos_next).magnitude;

                    Ray rayVector = new Ray(pos_start, pos_next);
                    RaycastHit hit;
                    if (Physics.Raycast(pos_start, direction, out hit))
                    {
                        if (hit.distance < maxLookAhead)
                        {
                            smoothedPath.PathPositions.Add(positions[j]);
                            //smoothedPath.PathNodes.Add(currentSolution.PathNodes[j]);
                            i = j;
                            break;
                        }
                        else
                        {
                            j++;
                        }
                    }
                    else
                    {
                        j++;
                    }
                }
                else {
                    smoothedPath.PathPositions.Add(positions[j]);
                    //smoothedPath.PathNodes.Add(currentSolution.PathNodes[j]);
                    i = j;
                    break;
                }
            }
        }
        /*while (i < nodes.len)
			j = i+1
			while (j < nodes.len)
				if (j+1 < nodes.len)
					pos_start = nodes[i].position
					pos_aux = nodes[j+1].position
					ray = ray(pos_start, pos_aux)
					if (ray.hit)
						smooth.add(nodes[j])
						i = j
						break
					else
						j++
				else
					smooth.add(nodes[j])
					i = j
					break
		*/
    }

    public void OnGUI()
    {
        if (this.currentSolution != null)
        {
            var time = this.AStarPathFinding.TotalProcessingTime * 1000;
            float timePerNode;
            if (this.AStarPathFinding.TotalExploredNodes > 0)
            {
                timePerNode = time / this.AStarPathFinding.TotalExploredNodes;
            }
            else
            {
                timePerNode = 0;
            }
            var text = "Nodes Visited: " + this.AStarPathFinding.TotalExploredNodes
                       + "\nMaximum Open Size: " + this.AStarPathFinding.MaxOpenNodes
                       + "\nProcessing time (ms): " + time.ToString("F")
                       + "\nTime per Node (ms):" + timePerNode.ToString("F4");

            GUI.contentColor = Color.black;
            GUI.Label(new Rect(10, 10, 300, 200), text);
        }
    }

    public void OnDrawGizmos()
    {
        if (this.draw)
        {
            //draw the current Solution Path if any (for debug purposes)
            if (this.currentSolution != null)
            {
                var previousPosition = this.startPosition;
                foreach (var pathPosition in this.currentSolution.PathPositions)
                {
                    Debug.DrawLine(previousPosition, pathPosition, Color.red);
                    previousPosition = pathPosition;
                }
            }

            //draw the smoothed Solution if any (for debug purposes)
            if (this.smoothedPath != null)
            {
                var previousPosition = this.startPosition;
                foreach (var pathPosition in this.smoothedPath.PathPositions)
                {
                    Debug.DrawLine(previousPosition, pathPosition, Color.green);
                    previousPosition = pathPosition;
                }
            }

            //draw the nodes in Open and Closed Sets
            if (this.AStarPathFinding != null)
            {
                Gizmos.color = Color.cyan;

                if (this.AStarPathFinding.Open != null)
                {
                    foreach (var nodeRecord in this.AStarPathFinding.Open.All())
                    {
                        Gizmos.DrawSphere(nodeRecord.node.LocalPosition, 1.0f);
                    }
                }

                Gizmos.color = Color.blue;

                if (this.AStarPathFinding.Closed != null)
                {
                    foreach (var nodeRecord in this.AStarPathFinding.Closed.All())
                    {
                        Gizmos.DrawSphere(nodeRecord.node.LocalPosition, 1.0f);
                    }
                }
            }
        }
    }

    private bool MouseClickPosition(out Vector3 position)
    {
        RaycastHit hit;

        var ray = this.camera.ScreenPointToRay(Input.mousePosition);
        //test intersection with objects in the scene
        if (Physics.Raycast(ray, out hit))
        {
            //if there is a collision, we will get the collision point
            position = hit.point;
            return true;
        }

        position = Vector3.zero;
        //if not the point is not valid
        return false;
    }

    private void InitializePathFinding(Vector3 p1, Vector3 p2)
    {

        //show the start sphere, hide the end one
        //this is just a small adjustment to better see the debug sphere
        this.startDebugSphere.transform.position = p1 + Vector3.up;
        this.startDebugSphere.SetActive(true);
        this.endDebugSphere.transform.position = p2 + Vector3.up;
        this.endDebugSphere.SetActive(true);
        this.currentClickNumber = 1;
        this.startPosition = p1;
        this.endPosition = p2;

        this.currentSolution = null;
        this.draw = true;

        this.AStarPathFinding.InitializePathfindingSearch(this.startPosition, this.endPosition);
    }
}