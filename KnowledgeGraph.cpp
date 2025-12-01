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
            delete edgeToRemove; // Free memory

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
}

template <class T>
void DGraphModel<T>::add(T vertex) {
    // TODO: Add a new vertex to the graph
}

template <class T>
void DGraphModel<T>::connect(T from, T to, float weight) {
    // TODO: Connect two vertices 'from' and 'to'

}

// TODO: Implement other methods of DGraphModel:

// =============================================================================
// Class KnowledgeGraph Implementation
// =============================================================================

KnowledgeGraph::KnowledgeGraph() {
    // TODO: Initialize the KnowledgeGraph
}

void KnowledgeGraph::addEntity(string entity) {
    // TODO: Add a new entity to the Knowledge Graph
}

void KnowledgeGraph::addRelation(string from, string to, float weight) {
    // TODO: Add a directed relation
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