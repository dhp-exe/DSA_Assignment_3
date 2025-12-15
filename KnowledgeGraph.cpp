#include "KnowledgeGraph.h"

// =============================================================================
// Class Edge Implementation
// =============================================================================

template <class T>
Edge<T>::Edge(VertexNode<T>* from, VertexNode<T>* to, float weight) {
    this->from = from;
    this->to = to;
    this->weight = weight;
}

template <class T>
string Edge<T>::toString() {
    stringstream ss;
    // Format: (<from>, <to>, <weight>)
    ss << "(";
    
    // If vertex2str exists, use it. Otherwise, print vertex directly.
    if (from) {
        if (from->vertex2str) ss << from->vertex2str(from->vertex);
        else ss << from->vertex;
    } 
    else {
        ss << "NULL";
    }
    
    ss << ", ";
    
    if (to) {
        if (to->vertex2str) ss << to->vertex2str(to->vertex);
        else ss << to->vertex;
    } 
    else {
        ss << "NULL";
    }
    
    ss.precision(6);
    ss.setf(std::ios::fixed, std::ios::floatfield);
    
    ss << ", " << weight << ")";
    return ss.str();
}
template <class T>
bool Edge<T>::equals(Edge<T>* edge) {
    if (edge == nullptr) return false;
    return (this->from == edge->from) && (this->to == edge->to);
}
template <class T>
bool Edge<T>::edgeEQ(Edge<T>*& edge1, Edge<T>*& edge2){
    if (edge1 == nullptr && edge2 == nullptr) return true;
    if (edge1 == nullptr || edge2 == nullptr) return false;
    return edge1->equals(edge2);
}

// =============================================================================
// Class VertexNode Implementation
// =============================================================================

template <class T>
VertexNode<T>::VertexNode(T vertex, bool (*vertexEQ)(T&, T&), string (*vertex2str)(T&)) {
    this->vertex = vertex;
    this->vertexEQ = vertexEQ;
    this->vertex2str = vertex2str;
    this->inDegree_ = 0;
    this->outDegree_ = 0;
}

template <class T>
void VertexNode<T>::connect(VertexNode<T>* to, float weight) {
    // Check if edge already exists (outgoing)
    for(Edge<T>* edge : adList) {
        if (edge->to == to && edge->from == this) {
            edge->weight = weight;
            return;
        }
    }
    
    // Create new edge
    Edge<T>* newEdge = new Edge<T>(this, to, weight);
    
    this->adList.push_back(newEdge); 
    to->adList.push_back(newEdge);   

    this->outDegree_++;
    to->inDegree_++;
}

template <class T>
T& VertexNode<T>::getVertex() {
    return this->vertex;
}

template<class T>
Edge<T>* VertexNode<T>::getEdge(VertexNode<T>* to) {
    for (Edge<T>* edge : adList) {
        if (edge->to == to && edge->from == this) {
            return edge;
        }
    }
    return nullptr;
}

template <class T>
bool VertexNode<T>::equals(VertexNode<T>* node){
    if(node == nullptr) return false;

    if(this->vertexEQ != nullptr){
        return this->vertexEQ(this->vertex, node->vertex);
    }
    return this->vertex == node->vertex;
}

template <class T>
void VertexNode<T>::removeTo(VertexNode<T>* to) {

    for (auto it = adList.begin(); it != adList.end(); it++) {
        if ((*it)->to == to && (*it)->from == this) {
            Edge<T>* edgeToRemove = *it;
            for(auto it2 = to->adList.begin(); it2 != to->adList.end(); ++it2) {
                if (*it2 == edgeToRemove) {
                    to->adList.erase(it2);
                    break;
                }
            }
            delete edgeToRemove; // Delete the actual edge object
            adList.erase(it);    // Remove from 'this' list
            
            this->outDegree_--;
            to->inDegree_--;
            return;
        }
    }
}

template <class T>
int VertexNode<T>::inDegree(){
    return this->inDegree_;
}

template <class T>
int VertexNode<T>::outDegree(){
    return this->outDegree_;
}

template <class T>
string VertexNode<T>::toString(){
    stringstream ss;
    // Format: (<value>, <in>, <out>, [<edges>])
    ss << "(";
    
    // If vertex2str exists, use it. Otherwise, print vertex directly.
    if (this->vertex2str) {
        ss << this->vertex2str(this->vertex);
    } else {
        ss << this->vertex;
    }
    
    ss << ", " << inDegree_ << ", " << outDegree_ << ", [";
    
    for (size_t i = 0; i < adList.size(); ++i) {
        if (i > 0) ss << ", ";
        ss << adList[i]->toString();
    }
    
    ss << "])";
    return ss.str();
}
// =============================================================================
// Class DGraphModel Implementation
// =============================================================================

template <class T>
DGraphModel<T>::DGraphModel(bool (*vertexEQ)(T&, T&), string (*vertex2str)(T&)) {
    this->vertexEQ = vertexEQ;
    this->vertex2str = vertex2str;
}

template <class T>
DGraphModel<T>::~DGraphModel() {
    // TODO: Clear all vertices and edges to avoid memory leaks
    this->clear();

}

template <class T>
void DGraphModel<T>::clear() {
    // Delete all Edges
    for (VertexNode<T>* node : nodeList) {
        for (Edge<T>* edge : node->adList) {
            if (edge->from == node) {
                delete edge;
            }
        }
        node->adList.clear(); // Clear the list pointers
    }

    // Delete all Vertices
    for (VertexNode<T>* node : nodeList) {
        delete node;
    }
    nodeList.clear();
}

template <class T>
void DGraphModel<T>::add(T vertex) {
    // TODO: Add a new vertex to the graph
    if (this->contains(vertex)) {
        return;
    }
    VertexNode<T>* newNode = new VertexNode<T>(vertex, this->vertexEQ, this->vertex2str);
    nodeList.push_back(newNode);
}

template <class T>
void DGraphModel<T>::connect(T from, T to, float weight) {
    // TODO: Connect two vertices 'from' and 'to'
    VertexNode<T>* fromNode = this->getVertexNode(from);
    VertexNode<T>* toNode = this->getVertexNode(to);

    if(!fromNode) throw VertexNotFoundException("Vertex not found");
    if(!toNode) throw VertexNotFoundException("Vertex not found");

    fromNode->connect(toNode, weight);
}

template <class T>
void DGraphModel<T>::disconnect(T from, T to) {
    VertexNode<T>* fromNode = this->getVertexNode(from);
    VertexNode<T>* toNode = this->getVertexNode(to);

    if(!fromNode) throw VertexNotFoundException("Vertex not found");
    if(!toNode) throw VertexNotFoundException("Vertex not found");

    fromNode->removeTo(toNode);
}

template <class T>
bool DGraphModel<T>::contains(T vertex) {
    return this->getVertexNode(vertex) != nullptr;
}

template <class T>
bool DGraphModel<T>::connected(T from, T to) {
    VertexNode<T>* fromNode = this->getVertexNode(from);
    VertexNode<T>* toNode = this->getVertexNode(to);
    if(!fromNode) throw VertexNotFoundException("Vertex not found");
    if(!toNode) throw VertexNotFoundException("Vertex not found");

    Edge<T>* edge = fromNode->getEdge(toNode);
    return edge != nullptr;
}

template <class T>
VertexNode<T>* DGraphModel<T>::getVertexNode(T& vertex) {
    for (VertexNode<T>* node : nodeList) {
        if(this->vertexEQ != nullptr){
            if(this->vertexEQ(node->getVertex(), vertex)){
                return node;
            }
        } 
        else {
            if (node->getVertex() == vertex) {
                return node;
            }
        }
    }
    return nullptr;
}

template <class T>
string DGraphModel<T>::vertex2Str(VertexNode<T>& node) {
    if (this->vertex2str != nullptr) {
        return this->vertex2str(node.getVertex());
    }
    return "";
}

template <class T>
string DGraphModel<T>::edge2Str(Edge<T>& edge) {
    return edge.toString();
}

template <class T>
float DGraphModel<T>::weight(T from, T to) {
    VertexNode<T>* fromNode = this->getVertexNode(from);
    VertexNode<T>* toNode = this->getVertexNode(to);

    if(!fromNode) throw VertexNotFoundException("Vertex not found");
    if(!toNode) throw VertexNotFoundException("Vertex not found");

    Edge<T>* edge = fromNode->getEdge(toNode);
    if(!edge) throw EdgeNotFoundException("Edge not found");

    return edge->weight;
}

template <class T>
vector<Edge<T>*> DGraphModel<T>::getOutwardEdges(T from) {
    VertexNode<T>* fromNode = this->getVertexNode(from);
    if(!fromNode) throw VertexNotFoundException("Vertex not found");

    // return only outgoing edges
    vector<Edge<T>*> outward;
    for (Edge<T>* edge : fromNode->adList) {
        if (edge->from == fromNode) {
            outward.push_back(edge);
        }
    }
    return outward;
}

template <class T>
int DGraphModel<T>::size() {
    return nodeList.size();
}

template <class T>
bool DGraphModel<T>::empty() {
    return nodeList.empty();
}

template <class T>
int DGraphModel<T>::inDegree(T vertex) {
    VertexNode<T>* node = this->getVertexNode(vertex);
    if(!node) throw VertexNotFoundException("Vertex not found");
    return node->inDegree();
}

template <class T>
int DGraphModel<T>::outDegree(T vertex) {
    VertexNode<T>* node = this->getVertexNode(vertex);
    if(!node) throw VertexNotFoundException("Vertex not found");
    return node->outDegree();
}

template <class T>
vector<T> DGraphModel<T>::vertices() {
    vector<T> v;
    for (VertexNode<T>* node : nodeList) {
        v.push_back(node->getVertex());
    }
    return v;
}

template <class T>
string DGraphModel<T>::toString() {
    stringstream ss;
    ss << "[";
    for (size_t i = 0; i < nodeList.size(); ++i) {
        if (i > 0) ss << ", ";
        ss << nodeList[i]->toString();
    }
    ss << "]";
    return ss.str();
}

// Format [<node1>, <node2>]
template <class T>
string DGraphModel<T>::BFS(T start) {
    VertexNode<T>* startNode = this->getVertexNode(start);
    if (!startNode) throw VertexNotFoundException("Start vertex not found");

    vector<VertexNode<T>*> visited;
    vector<VertexNode<T>*> queue; 
    queue.push_back(startNode);
    visited.push_back(startNode);
    
    stringstream ss;
    ss << "[";
    
    int head = 0;
    bool first = true;
    while(head < queue.size()){
        VertexNode<T>* current = queue[head++];
        
        if (!first) ss << ", ";

        if(this->vertex2str != nullptr){
            ss << this->vertex2str(current->vertex);
        }
        else{
            ss << current->toString();
        }

        first = false;

        for (Edge<T>* edge : current->adList) {

            if (edge->from != current) continue; // only consider outward edges

            VertexNode<T>* neighbor = edge->to;
            bool seen = false;
            for(auto v : visited) if(v == neighbor) seen = true;
            
            if (!seen) {
                visited.push_back(neighbor);
                queue.push_back(neighbor);
            }
        }
    }
    ss << "]";
    return ss.str();
}

// Format [<node1>, <node2>]
template <class T>
string DGraphModel<T>::DFS(T start) {
    VertexNode<T>* startNode = this->getVertexNode(start);
    if (!startNode) throw VertexNotFoundException("Start vertex not found");

    vector<VertexNode<T>*> visited;
    vector<VertexNode<T>*> stack;
    stack.push_back(startNode);
    
    stringstream ss;
    ss << "[";
    bool first = true;

    while(!stack.empty()){
        VertexNode<T>* current = stack.back();
        stack.pop_back();

        bool seen = false;
        for(auto v : visited) if(v == current) seen = true;

        if (!seen) {
            visited.push_back(current);
            if (!first) ss << ", ";

            if(this->vertex2str != nullptr){
                ss << this->vertex2str(current->vertex);
            }
            else{
                ss << current->toString(); 
            }

            first = false;

            // Push neighbors in reverse to maintain processing order
            for (int i = current->adList.size() - 1; i >= 0; i--) {
                Edge<T>* edge = current->adList[i];
                
                if (edge->from != current) continue; 

                VertexNode<T>* neighbor = edge->to;
                bool seenNeighbor = false;
                for(auto v : visited) if(v == neighbor) seenNeighbor = true;
                
                if (!seenNeighbor) {
                    stack.push_back(neighbor);
                }
            }
        }
    }
    ss << "]";
    return ss.str();
}
// =============================================================================
// Class KnowledgeGraph Implementation
// =============================================================================

KnowledgeGraph::KnowledgeGraph() {
    // TODO: Initialize the KnowledgeGraph
}

void KnowledgeGraph::addEntity(string entity) {
    // TODO: Add a new entity to the Knowledge Graph
    if (graph.contains(entity)) {
        throw EntityExistsException("Entity already exists!");
    }
    graph.add(entity);
    entities.push_back(entity);
}

void KnowledgeGraph::addRelation(string from, string to, float weight) {
    // TODO: Add a directed relation
    if (!graph.contains(from) || !graph.contains(to)) {
        throw EntityNotFoundException("Entity not found!");
    }
    graph.connect(from, to, weight);
}

vector<string> KnowledgeGraph::getAllEntities() {
    return entities;
}

vector<string> KnowledgeGraph::getNeighbors(string entity) {
    if (!graph.contains(entity)) {
        throw EntityNotFoundException("Entity not found!");
    }
    
    // Get the list of Edge pointers
    vector<Edge<string>*> edges = graph.getOutwardEdges(entity);
    vector<string> neighbors;
    
    for (Edge<string>* edge : edges) {
        // Use the public getter instead of direct access
        neighbors.push_back(edge->getTo()->getVertex());
    }
    
    return neighbors;
}

string KnowledgeGraph::bfs(string start) {
    if (!graph.contains(start)) throw EntityNotFoundException("Entity not found");
    return graph.BFS(start);
}

string KnowledgeGraph::dfs(string start) {
    if (!graph.contains(start)) throw EntityNotFoundException("Entity not found");
    return graph.DFS(start);
}

bool KnowledgeGraph::isReachable(string from, string to) {
    if (!graph.contains(from) || !graph.contains(to)) {
        throw EntityNotFoundException("Entity not found!");
    }
    vector<string> q;
    vector<string> visited;
    q.push_back(from);
    visited.push_back(from);

    int head = 0;
    while (head < q.size()) {
        string current = q[head];
        head++;
        if (current == to) return true;

        vector<Edge<string>*> edges = graph.getOutwardEdges(current);
        for (Edge<string>* edge : edges) {
            string neighbor = edge->getTo()->getVertex();
            bool seen = false;
            for (const string& v : visited) {
                if (v == neighbor) {
                    seen = true;
                    break;
                }
            }
            if (!seen) {
                visited.push_back(neighbor);
                q.push_back(neighbor);
            }
        }
    }
    return false;
}
string KnowledgeGraph::toString() {
    return graph.toString();
}

vector<string> KnowledgeGraph::getRelatedEntities(string entity, int depth) {
    if (!graph.contains(entity)) throw EntityNotFoundException("Entity not found");
    
    vector<string> result;
    vector<pair<string, int>> q; //used as queue
    vector<string> visited;
    
    q.push_back({entity, 0});
    visited.push_back(entity);
    
    int head = 0;
    while(head < q.size()){
        pair<string, int> current = q[head++];
        string name = current.first;
        int d = current.second;
        
        if(d > 0) result.push_back(name); // Add if related (depth > 0)
        
        if(d < depth){
            vector<Edge<string>*> edges = graph.getOutwardEdges(name);
            for (Edge<string>* edge : edges) {
                string n = edge->getTo()->getVertex();
                bool seen = false;
                for(string v : visited) if(v == n) seen = true;
                
                if(!seen){
                    visited.push_back(n);
                    q.push_back({n, d + 1});
                }
            }
        }
    }
    return result;
}

// Helper: check if vector contains a value
static bool kg_containsVec(const vector<string> &vec, const string &val) {
    for (size_t i = 0; i < vec.size(); ++i) if (vec[i] == val) return true;
    return false;
}

// Helper: get direct parents (incoming neighbors) of a node by scanning entities
static vector<string> kg_getParents(const vector<string> &entities, DGraphModel<string> &graph, const string &target) {
    vector<string> parents;
    for (size_t i = 0; i < entities.size(); ++i) {
        const string &potentialParent = entities[i];
        if (graph.connected(potentialParent, target)) parents.push_back(potentialParent);
    }
    return parents;
}

// Collect all ancestors using Reverse DFS (Stack-based)
static vector<string> kg_collectAncestors(const vector<string> &entities, DGraphModel<string> &graph, const string &start) {
    vector<string> ancestors; // visited ancestors
    vector<string> stack;     // Stack for DFS
    
    // 1. Get immediate parents of the start node to begin traversal
    vector<string> firstParents = kg_getParents(entities, graph, start);
    for(size_t i = 0; i < firstParents.size(); ++i) {
        string p = firstParents[i];
        if (!kg_containsVec(ancestors, p)) {
            ancestors.push_back(p);
            stack.push_back(p);
        }
    }
    
    // 2. Standard DFS Loop
    while (!stack.empty()) {
        // Pop from the back (LIFO - Last In First Out)
        string curr = stack.back();
        stack.pop_back();
        
        // Find parents of the current ancestor
        vector<string> pars = kg_getParents(entities, graph, curr);
        for (size_t i = 0; i < pars.size(); ++i) {
            string p = pars[i];
            
            // If not visited, mark visited and push to stack
            if (!kg_containsVec(ancestors, p)) {
                ancestors.push_back(p);
                stack.push_back(p);
            }
        }
    }
    return ancestors;
}

// Helper: get index of entity in entities vector
static int kg_indexOf(const vector<string> &entities, const string &val) {
    for (size_t i = 0; i < entities.size(); ++i) if (entities[i] == val) return (int)i;
    return -1;
}

// Dijkstra-like shortest path (by weight) using simple arrays (no extra libraries)
static double kg_shortestPath(const vector<string> &entities, DGraphModel<string> &graph, const string &src, const string &dst) {
    const double INF = HUGE_VAL;
    int n = (int)entities.size();
    if (n == 0) return INF;
    vector<double> dist(n, INF);
    vector<int> used(n, 0);
    int sidx = kg_indexOf(entities, src);
    int didx = kg_indexOf(entities, dst);
    if (sidx < 0 || didx < 0) return INF;
    dist[sidx] = 0.0;

    for (int iter = 0; iter < n; ++iter) {
        int u = -1;
        double best = INF;
        for (int i = 0; i < n; ++i) {
            if (!used[i] && dist[i] < best) {
                best = dist[i];
                u = i;
            }
        }
        if (u == -1) break;
        if (u == didx) return dist[u];
        used[u] = 1;

        vector<Edge<string>*> edges = graph.getOutwardEdges(entities[u]);
        for (size_t ei = 0; ei < edges.size(); ++ei) {
            Edge<string>* e = edges[ei];
            string v = e->getTo()->getVertex();
            int vidx = kg_indexOf(entities, v);
            if (vidx < 0 || used[vidx]) continue;
            double nd = dist[u] + e->getWeight();
            if (nd < dist[vidx]) dist[vidx] = nd;
        }
    }
    return INF;
}

string KnowledgeGraph::findCommonAncestors(string entity1, string entity2) {
    // 1. Check existence
    if (!graph.contains(entity1) || !graph.contains(entity2)) {
        throw EntityNotFoundException("Entity not found");
    }

    // 2. Find all ancestors using Reverse DFS
    vector<string> ancestors1 = kg_collectAncestors(entities, graph, entity1);
    vector<string> ancestors2 = kg_collectAncestors(entities, graph, entity2);

    // 3. Find Intersection
    vector<string> common;
    for (size_t i = 0; i < ancestors1.size(); ++i) {
        if (kg_containsVec(ancestors2, ancestors1[i])) {
            common.push_back(ancestors1[i]);
        }
    }

    if (common.empty()) return "No common ancestor";

    // 4. Find Best Ancestor (Shortest Path to both)
    const double INF = HUGE_VAL;
    double bestScore = INF;
    string bestAncestor = "";

    for (size_t i = 0; i < common.size(); ++i) {
        string cand = common[i];
        
        // Calculate distances from Ancestor -> Entity1 and Ancestor -> Entity2
        double d1 = kg_shortestPath(entities, graph, cand, entity1);
        double d2 = kg_shortestPath(entities, graph, cand, entity2);
        
        if (d1 == INF || d2 == INF) continue; 
        
        double total = d1 + d2;
        if (total < bestScore) {
            bestScore = total;
            bestAncestor = cand;
        }
    }

    if (bestAncestor.empty()) return "No common ancestor";
    return bestAncestor;
}

// =============================================================================
// Explicit Template Instantiation
// =============================================================================

template class Edge<string>;
template class Edge<int>;
template class Edge<float>;
template class Edge<char>;

template class VertexNode<string>;
template class VertexNode<int>;
template class VertexNode<float>;
template class VertexNode<char>;

template class DGraphModel<string>;
template class DGraphModel<int>;
template class DGraphModel<float>;
template class DGraphModel<char>;