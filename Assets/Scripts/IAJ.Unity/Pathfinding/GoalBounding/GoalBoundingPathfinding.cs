using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding;
using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using RAIN.Navigation.NavMesh;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using RAIN.Navigation.Graph;
using UnityEngine;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.GoalBounding
{
    public class GoalBoundingPathfinding : NodeArrayAStarPathFinding
    {
        public GoalBoundingTable goalBoundingTable { get; protected set; }
        public int DiscardedEdges { get; protected set; }
        public int TotalEdges { get; protected set; }

        public GoalBoundingPathfinding(NavMeshPathGraph graph, IHeuristic heuristic, GoalBoundingTable goalBoundsTable) : base(graph, heuristic)
        {
            this.goalBoundingTable = goalBoundsTable;
        }

        public override void InitializePathfindingSearch(Vector3 startPosition, Vector3 goalPosition)
        {
            this.DiscardedEdges = 0;
            this.TotalEdges = 0;
            base.InitializePathfindingSearch(startPosition, goalPosition);
        }

        //protected override void ProcessChildNode(NodeRecord parentNode, NavigationGraphEdge connectionEdge, int edgeIndex)
        protected override void ProcessChildNode(NodeRecord parentNode, NavigationGraphEdge connectionEdge, int edgeIndex)
        {
            //TODO: Implement this method for the GoalBoundingPathfinding to Work. If you implemented the NodeArrayAStar properly, you wont need to change the search method.
            float f;
            float g;
            float h;

            var childNode = connectionEdge.ToNode;
            var childNodeRecord = this.NodeRecordArray.GetNodeRecord(childNode);

            if (childNodeRecord == null)
            {
                //this piece of code is used just because of the special start nodes and goal nodes added to the RAIN Navigation graph when a new search is performed.
                //Since these special goals were not in the original navigation graph, they will not be stored in the NodeRecordArray and we will have to add them
                //to a special structure
                //it's ok if you don't understand this, this is a hack and not part of the NodeArrayA* algorithm, just do NOT CHANGE THIS, or your algorithm will not work
                childNodeRecord = new NodeRecord
                {
                    node = childNode,
                    parent = parentNode,
                    status = NodeStatus.Unvisited
                };
                this.NodeRecordArray.AddSpecialCaseNode(childNodeRecord);
            }

            //TODO: implement the rest of your code here

            

            //indice da cor do rectangulo
            var color = edgeIndex;
            //indice startNode
            var startNode = parentNode.node.NodeIndex;

            //entrada da tabela dos rectangulos
            //var bbox = this.goalBoundingTable.table[color].connectionBounds[startNode];
            bool inBounds = false;

            if (this.goalBoundingTable.table[startNode] != null)
            {
                var nodeBounds = this.goalBoundingTable.table[startNode];
                if(color < nodeBounds.connectionBounds.Length)
                {
                    var bbox = nodeBounds.connectionBounds[color];
                    inBounds = bbox.PositionInsideBounds(childNodeRecord.node.Position);
                }
                else
                {
                    inBounds = true;
                }
            }
            else
            {
                inBounds = true;
            }

            if(!inBounds )
            {
                return;
            }
            base.ProcessChildNode(parentNode, connectionEdge, edgeIndex);

        }
    }
}
