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
    ss << (from ? (from->vertex2str ? from->vertex2str(from->vertex) : "NULL") : "NULL"); // Handle from
    ss << ", ";
    ss << (to ? (to->vertex2str ? to->vertex2str(to->vertex) : "NULL") : "NULL");       // Handle to
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
        if (edge->to == to) {
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
    ss << (vertex2str ? vertex2str(vertex) : ""); // Print value
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

string KnowledgeGraph::findCommonAncestors(string entity1, string entity2) {
    if (!graph.contains(entity1) || !graph.contains(entity2)) {
        throw EntityNotFoundException("Entity not found");
    }
    
    // Helper: get direct parents (incoming neighbors) of a node by scanning entities
    auto getParents = [&](const string &target) -> vector<string> {
        vector<string> parents;
        for (const string &potentialParent : entities) {
            if (graph.connected(potentialParent, target)) {
                parents.push_back(potentialParent);
            }
        }
        return parents;
    };

    // Collect all ancestors (reverse BFS) for a start node. Excludes the start node itself.
    auto collectAncestors = [&](const string &start) -> unordered_set<string> {
        unordered_set<string> ancestors;
        vector<string> q;
        q.push_back(start);
        size_t head = 0;
        while (head < q.size()) {
            string curr = q[head++];
            vector<string> pars = getParents(curr);
            for (const string &p : pars) {
                if (ancestors.insert(p).second) {
                    q.push_back(p);
                }
            }
        }
        return ancestors;
    };

    unordered_set<string> ancestors1 = collectAncestors(entity1);
    unordered_set<string> ancestors2 = collectAncestors(entity2);

    // Intersection of ancestor sets
    vector<string> common;
    for (const string &a : ancestors1) {
        if (ancestors2.count(a)) common.push_back(a);
    }
    if (common.empty()) return "No common ancestor";

    // Dijkstra shortest path (by weight) from src -> dst. Returns +inf if unreachable.
    auto shortestPath = [&](const string &src, const string &dst) -> double {
        const double INF = std::numeric_limits<double>::infinity();
        unordered_map<string, double> dist;
        for (const string &v : entities) dist[v] = INF;

        // min-heap by distance
        auto cmp = [](const pair<double, string> &a, const pair<double, string> &b) { return a.first > b.first; };
        priority_queue<pair<double, string>, vector<pair<double, string>>, decltype(cmp)> pq(cmp);

        dist[src] = 0.0;
        pq.push({0.0, src});

        while (!pq.empty()) {
            auto cur = pq.top(); pq.pop();
            double d = cur.first;
            const string u = cur.second;
            if (d > dist[u]) continue;
            if (u == dst) return d;

            vector<Edge<string>*> edges = graph.getOutwardEdges(u);
            for (Edge<string>* e : edges) {
                string v = e->getTo()->getVertex();
                double nd = d + e->getWeight();
                if (nd < dist[v]) {
                    dist[v] = nd;
                    pq.push({nd, v});
                }
            }
        }
        return std::numeric_limits<double>::infinity();
    };

    const double INF = std::numeric_limits<double>::infinity();
    double bestScore = INF;
    string bestAncestor = "";

    // For each common ancestor, compute sum of shortest-path weights to both entities
    for (const string &cand : common) {
        double d1 = shortestPath(cand, entity1);
        double d2 = shortestPath(cand, entity2);
        if (d1 == INF || d2 == INF) continue; // must reach both
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