% This function is used to find an optimum path using Dijkstra's algorithm
% Inputs: Network Graph, SourceNode, TerminalNode are given to the function
% Outputs: Route is the final best path from the given network graph

function Route = Dijkstras( Graph, SourceNode, TerminalNode )
%Dijkstras.m Given a graph with distances from node to node calculates the
%optimal route from the Source Node to the Terminal Node as defined by the
%inputs.

% Special Case so no need to waste time doing initializations
if SourceNode == TerminalNode
    Cost = Graph(SourceNode, TerminalNode);
    Route = SourceNode;
    return;
end

% Set up a cell structure so that I can store the optimal path from source 
% node to each node in this structure. This structure stores the
% antecedents so for instance if there is a path to B through A-->C-->D-->B
% you will see [A,C,D] in cell{B} (as well as a bunch of filler 0's after
% that)
PathToNode = cell(size(Graph,1),1);

% Initialize all Node costs to infinity except for the source node
NodeCost = Inf.*ones(1,size(Graph,1));
NodeCost(SourceNode) = 0;

% Initialize the Current Node to be the Source Node
CurrentNode = SourceNode;

% Initialize the set of Visited and Unvisited Nodes
VisitedNodes = SourceNode;
UnvisitedNodes = 1:size(Graph,2);
UnvisitedNodes = UnvisitedNodes(UnvisitedNodes ~= VisitedNodes);

while (CurrentNode ~= TerminalNode)
    % Extract the Costs/Path Lengths to each node from the current node
    CostVector = Graph(CurrentNode, :);
    % Only look at valid neighbors ie. those nodes which are unvisited
    UnvisitedNeighborsCostVector = CostVector(UnvisitedNodes);
    % Extract the cost to get to the Current Node
    CurrentNodeCost = NodeCost(CurrentNode);
    % Extract the path to the current node
    PathToCurrentNode = PathToNode{CurrentNode};
    % Iterate through the Unvisited Neighbors assigning them a new tentative cost
    for i = 1:length(UnvisitedNeighborsCostVector)
       if UnvisitedNeighborsCostVector(i) ~= Inf % Only Check for update if non-infinite
           tempCost = CurrentNodeCost + UnvisitedNeighborsCostVector(i); % The tentative cost to get to the neighbor through the current node
           % Compare the tentative cost to the currently assigned cost and
           % assign the minimum
           if tempCost < NodeCost(UnvisitedNodes(i))
               NewPathToNeighbor = [PathToCurrentNode(PathToCurrentNode~=0) CurrentNode]; % The new path to get to the neighbor
               NewPath = [NewPathToNeighbor zeros(1,size(Graph,1)-size(NewPathToNeighbor,2))];
               PathToNode{UnvisitedNodes(i)}(:) = NewPath;
               NodeCost(UnvisitedNodes(i)) = tempCost;
           end
       end
    end
    % Search for the smallest cost remaining that is in the unvisited set
    RemainingCosts = NodeCost(UnvisitedNodes);
    [MIN, MIN_IND] = min(RemainingCosts);
    
    % If the smallest remaining cost amongst the unvisited set of nodes is
    % infinite then there is no valid path from the source node to the
    % terminal node. 
    if MIN == Inf
       fprintf('There is no valid path from the source node to the');
       fprintf('terminal node. Please check your graph.\n')
       return;
    end
    
    % Update the Visited and Unvisited Nodes
    VisitedNodes = [VisitedNodes CurrentNode];
    CurrentNode = UnvisitedNodes(MIN_IND);
    UnvisitedNodes = UnvisitedNodes(UnvisitedNodes~=CurrentNode);
end

Route = PathToNode{TerminalNode};
Route = Route(Route~=0);
Route = [Route TerminalNode];
Cost = NodeCost(TerminalNode);
end