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
    // TODO: Return the string representation of the edge
    return "";
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
    // TODO: Connect this vertex to the 'to' vertex
    // Edge already exists, update weight
    for(Edge<T>* edge : adList) {
        if (edge->to == to) {
            edge->weight = weight;
            return;
        }
    }
    // Create new edge
    Edge<T>* newEdge = new Edge<T>(this, to, weight);
    adList.push_back(newEdge);

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
        if ((*it)->to == to) {
            Edge<T>* edgeToRemove = *it;
            delete edgeToRemove;

            adList.erase(it);
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
    for (VertexNode<T>* node : nodeList) {
        for (Edge<T>* edge : node->adList) {
            delete edge;
        }
        node->adList.clear();
    }
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
    if(!fromNode) throw VertexNotFoundException(this->vertex2Str(*fromNode));
    if(!toNode) throw VertexNotFoundException(this->vertex2Str(*toNode));

    Edge<T>* edge = fromNode->getEdge(toNode);
    return edge != nullptr;
}

template <class T>
VertexNode<T>* DGraphModel<T>::getVertexNode(T& vertex) {
    for (VertexNode<T>* node : nodeList) {
        if(this->vertexEq != nullptr){
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
vector<T> DGraphModel<T>::getOutwardEdges(T from) {
    VertexNode<T>* fromNode = this->getVertexNode(from);
    if(!fromNode) throw VertexNotFoundException("Vertex not found");

    vector<T> neighbors;
    for (Edge<T>* edge : fromNode->adList) {
        neighbors.push_back(edge->to->getVertex());
    }
    return neighbors;
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
}

template <class T>
string DGraphModel<T>::BFS(T start) {
    VertexNode<T>* startNode = this->getVertexNode(start);
    if (!startNode) throw VertexNotFoundException("Start vertex not found");

    vector<VertexNode<T>*> visited;
    vector<VertexNode<T>*> queue; // using vector as queue
    
    queue.push_back(startNode);
    visited.push_back(startNode);
    
    stringstream ss;
    bool first = true;
    int head = 0; 

    while(head < queue.size()){
        VertexNode<T>* current = queue[head];
        head++; // Pop front

        if (!first) ss << " ";
        ss << this->vertex2Str(*current);
        first = false;

        for (Edge<T>* edge : current->adList) {
            VertexNode<T>* neighbor = edge->to;
            
            bool seen = false;
            for(VertexNode<T>* v : visited) {
                if(v == neighbor) {
                    seen = true; 
                    break;
                }
            }

            if (!seen) {
                visited.push_back(neighbor);
                queue.push_back(neighbor);
            }
        }
    }
    return ss.str();
}

template <class T>
string DGraphModel<T>::DFS(T start) {
    VertexNode<T>* startNode = this->getVertexNode(start);
    if (!startNode) throw VertexNotFoundException("Start vertex not found");

    vector<VertexNode<T>*> visited;
    vector<VertexNode<T>*> stack; // using vector as stack
    
    stack.push_back(startNode);
    
    stringstream ss;
    bool first = true;

    while(!stack.empty()){
        VertexNode<T>* current = stack.back();
        stack.pop_back();

        // Check visited
        bool seen = false;
        for(VertexNode<T>* v : visited) {
            if(v == current) {
                seen = true;
                break;
            }
        }

        if (!seen) {
            visited.push_back(current);
            
            if (!first) ss << " ";
            ss << this->vertex2Str(*current);
            first = false;

            // Push neighbors to stack in REVERSE order 
            // so the first neighbor is at the top of the stack
            for (int i = current->adList.size() - 1; i >= 0; i--) {
                stack.push_back(current->adList[i]->to);
            }
        }
    }
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
    return graph.vertices();
}

vector<string> KnowledgeGraph::getNeighbors(string entity) {
    if (!graph.contains(entity)) {
        throw EntityNotFoundException("Entity not found!");
    }
    return graph.getOutwardEdges(entity);
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

        vector<string> neighbors = graph.getOutwardEdges(current);
        for (const string& neighbor : neighbors) {
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
// TODO: Implement other methods of KnowledgeGraph:



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