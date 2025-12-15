#include "KnowledgeGraph.h"
#include <iostream>
#include <string>
#include <vector>

using namespace std;

// =============================================================================
// HELPER FUNCTIONS FOR TESTING
// =============================================================================
bool intEQ(int& a, int& b) { return a == b; }
string int2str(int& i) { return to_string(i); }

bool strEQ(string& a, string& b) { return a == b; }
string str2str(string& s) { return s; }

// =============================================================================
// 1. Edge Class Tests
// =============================================================================
void testEdgeClass() {
    cout << "\n==================================================" << endl;
    cout << "TESTING CLASS: Edge" << endl;
    cout << "==================================================" << endl;

    // Setup dummy nodes
    VertexNode<string>* nodeA = new VertexNode<string>("NodeA", strEQ, str2str);
    VertexNode<string>* nodeB = new VertexNode<string>("NodeB", strEQ, str2str);

    // 1. Constructor
    cout << "\n[1] Creating Edge(NodeA, NodeB, 5.5)..." << endl;
    Edge<string> edge1(nodeA, nodeB, 5.5f);

    // 2. toString
    cout << "edge1.toString(): " << edge1.toString() << endl; 
    // Expected: (NodeA, NodeB, 5.500000)

    // 3. equals
    cout << "\n[3] Testing equals():" << endl;
    Edge<string> edge2(nodeA, nodeB, 10.0f); // Same nodes, diff weight
    Edge<string> edge3(nodeB, nodeA, 5.5f);  // Diff direction
    
    cout << "edge1.equals(&edge2): " << (edge1.equals(&edge2) ? "TRUE" : "FALSE") << endl; 
    // Expected: TRUE (Same src/dst)
    
    cout << "edge1.equals(&edge3): " << (edge1.equals(&edge3) ? "TRUE" : "FALSE") << endl; 
    // Expected: FALSE (Diff direction)

    // 4. static edgeEQ
    cout << "\n[4] Testing static edgeEQ():" << endl;
    
    // [FIX] Create pointer variables first. 
    // You cannot pass '&edge1' directly because the function takes a reference (Edge*&)
    Edge<string>* ptr1 = &edge1;
    Edge<string>* ptr2 = &edge2;
    
    cout << "Edge::edgeEQ(ptr1, ptr2): " << (Edge<string>::edgeEQ(ptr1, ptr2) ? "TRUE" : "FALSE") << endl;
    // Expected: TRUE

    // Cleanup manually allocated nodes for this specific test
    delete nodeA;
    delete nodeB;
}

// =============================================================================
// 2. VertexNode Class Tests
// =============================================================================
void testVertexNodeClass() {
    cout << "\n==================================================" << endl;
    cout << "TESTING CLASS: VertexNode" << endl;
    cout << "==================================================" << endl;

    VertexNode<string> v1("V1", strEQ, str2str);
    VertexNode<string> v2("V2", strEQ, str2str);
    VertexNode<string> v3("V3", strEQ, str2str);

    // 1. getVertex
    cout << "\n[1] getVertex(): " << v1.getVertex() << endl; 
    // Expected: V1

    // 2. connect
    cout << "\n[2] Connecting V1->V2 (w=1.0) and V2->V3 (w=2.0)..." << endl;
    v1.connect(&v2, 1.0f);
    v2.connect(&v3, 2.0f);
    cout<<" V1 InDegree:  " << v1.inDegree() << endl; 
    // Expected: 0
    cout << "V1 OutDegree: " << v1.outDegree() << endl; 
    // Expected: 1
    cout << "V2 InDegree:  " << v2.inDegree() << endl;  
    // Expected: 1
    cout << "V2 OutDegree: " << v2.outDegree() << endl; 
    // Expected: 1

    // 3. getEdge
    cout << "\n[3] getEdge(V1 -> V2):" << endl;
    Edge<string>* e = v1.getEdge(&v2);
    Edge<string>* e2 = v1.getEdge(&v3); // Non-existing edge
    if(e2) cout << "Result: " << e2->toString() << endl; 
    else    cout << "Result: NULL" << endl;
    // Expected: NULL
    if (e) cout << "Result: " << e->toString() << endl; 
    else   cout << "Result: NULL" << endl;
    
    // Expected: (V1, V2, 1.000000)

    // 4. equals
    cout << "\n[4] equals():" << endl;
    cout << "v1.equals(&v1): " << (v1.equals(&v1) ? "TRUE" : "FALSE") << endl; 
    // Expected: TRUE
    cout << "v1.equals(&v2): " << (v1.equals(&v2) ? "TRUE" : "FALSE") << endl; 
    // Expected: FALSE

    // 5. toString
    cout << "\n[5] toString():" << endl;
    cout << "V1: " << v1.toString() << endl; 
    // Expected: (V1, 0, 1, [(V1, V2, 1.000000)])
    cout << "V2: " << v2.toString() << endl; 
    // Expected: (V2, 1, 1, [(V1, V2, 1.000000), (V2, V3, 2.000000)]) 

    // 6. removeTo
    cout << "\n[6] removeTo(V2) from V1..." << endl;
    v1.removeTo(&v2);
    cout << "V1 OutDegree after remove: " << v1.outDegree() << endl; 
    // Expected: 0
    cout << "V1 toString after remove:  " << v1.toString() << endl; 
    // Expected: (V1, 0, 0, [])
}

// =============================================================================
// 3. DGraphModel Class Tests
// =============================================================================
void testDGraphModelClass() {
    cout << "\n==================================================" << endl;
    cout << "TESTING CLASS: DGraphModel" << endl;
    cout << "==================================================" << endl;

    DGraphModel<string> graph(strEQ, str2str);

    // 1. add / size / empty
    cout << "\n[1] Checking empty() and adding vertices A, B, C, D..." << endl;
    cout << "Initially empty? " << (graph.empty() ? "YES" : "NO") << endl; 
    // Expected: YES
    
    graph.add("A"); graph.add("B"); graph.add("C"); graph.add("D");
    cout << "Size after adding 4: " << graph.size() << endl; 
    // Expected: 4

    // 2. contains
    cout << "\n[2] contains():" << endl;
    cout << "Contains 'A'? " << (graph.contains("A") ? "YES" : "NO") << endl; 
    // Expected: YES
    cout << "Contains 'Z'? " << (graph.contains("Z") ? "YES" : "NO") << endl; 
    // Expected: NO

    // 3. connect / connected / weight
    cout << "\n[3] Connecting A->B (1.5), B->C (2.0), A->C (3.0)..." << endl;
    graph.connect("A", "B", 1.5f);
    graph.connect("B", "C", 2.0f);
    graph.connect("A", "C", 3.0f); 
    
    cout << "Connected A->B? " << (graph.connected("A", "B") ? "YES" : "NO") << endl; 
    // Expected: YES
    cout << "Weight A->B:    " << graph.weight("A", "B") << endl; 
    // Expected: 1.5
    cout << "Connected B->A? " << (graph.connected("B", "A") ? "YES" : "NO") << endl; 
    // Expected: NO

    // 4. getOutwardEdges
    cout << "\n[4] getOutwardEdges('A'):" << endl;
    try {
        vector<Edge<string>*> edges = graph.getOutwardEdges("A");
        cout << "Edge count: " << edges.size() << endl; 
        // Expected: 2 (A->B, A->C)
        for(auto e : edges) cout << "  - " << e->toString() << endl;
    } catch(exception& e) { cout << e.what() << endl; }

    // 5. Degrees
    cout << "\n[5] Degrees for 'B':" << endl;
    cout << "InDegree:  " << graph.inDegree("B") << endl; 
    // Expected: 1
    cout << "OutDegree: " << graph.outDegree("B") << endl; 
    // Expected: 1

    // 6. toString (Graph)
    cout << "\n[6] DGraphModel::toString():" << endl;
    cout << graph.toString() << endl;
    // Expected: String representation of all nodes

    // 7. BFS / DFS
    cout << "\n[7] Traversals starting from 'A':" << endl;
    cout << "BFS: " << graph.BFS("A") << endl; 
    // Expected: [A, B, C] or similar
    cout << "DFS: " << graph.DFS("A") << endl; 
    // Expected: [A, C, B] or similar

    // 8. disconnect
    cout << "\n[8] disconnect('A', 'B')..." << endl;
    graph.disconnect("A", "B");
    cout << "Connected A->B now? " << (graph.connected("A", "B") ? "YES" : "NO") << endl; 
    // Expected: NO

    // 9. vertices
    cout << "\n[9] vertices():" << endl;
    vector<string> allV = graph.vertices();
    cout << "Vertices: ";
    for(auto v : allV) cout << v << " ";
    cout << endl;
    // Expected: A B C D 

    // 10. clear
    cout << "\n[10] clear()..." << endl;
    graph.clear();
    cout << "Size after clear: " << graph.size() << endl; 
    // Expected: 0
}

// =============================================================================
// 4. KnowledgeGraph Class Tests
// =============================================================================
void testKnowledgeGraphClass() {
    cout << "\n==================================================" << endl;
    cout << "TESTING CLASS: KnowledgeGraph" << endl;
    cout << "==================================================" << endl;

    KnowledgeGraph kg;

    // 1. addEntity
    cout << "\n[1] Adding entities: Grandpa, Dad, Mom, Son, Daughter, Stranger..." << endl;
    kg.addEntity("Grandpa");
    kg.addEntity("Dad");
    kg.addEntity("Mom");
    kg.addEntity("Son");
    kg.addEntity("Daughter");
    kg.addEntity("Stranger");

    // 2. addRelation
    cout << "\n[2] Adding relations..." << endl;
    kg.addRelation("Grandpa", "Dad");
    kg.addRelation("Dad", "Son");
    kg.addRelation("Dad", "Daughter");
    kg.addRelation("Mom", "Son");
    kg.addRelation("Mom", "Daughter");
    // Removed Cycle for standard tests to keep things simple
    // kg.addRelation("Son", "Mom"); 

    // 3. getAllEntities
    cout << "\n[3] getAllEntities():" << endl;
    vector<string> ents = kg.getAllEntities();
    cout << "Count: " << ents.size() << endl; 
    // Expected: 6

    // 4. getNeighbors
    cout << "\n[4] getNeighbors('Dad'):" << endl;
    vector<string> neighbors = kg.getNeighbors("Dad");
    cout << "Neighbors: ";
    for(auto n : neighbors) cout << n << " ";
    cout << endl;
    // Expected: Son Daughter

    // 5. isReachable
    cout << "\n[5] isReachable():" << endl;
    cout << "Grandpa -> Son:      " << (kg.isReachable("Grandpa", "Son") ? "YES" : "NO") << endl; 
    // Expected: YES
    cout << "Son -> Grandpa:      " << (kg.isReachable("Son", "Grandpa") ? "YES" : "NO") << endl; 
    // Expected: NO
    cout << "Grandpa -> Stranger: " << (kg.isReachable("Grandpa", "Stranger") ? "YES" : "NO") << endl; 
    // Expected: NO

    // 6. BFS / DFS Wrappers
    cout << "\n[6] BFS/DFS Wrappers (start 'Grandpa'):" << endl;
    cout << "BFS: " << kg.bfs("Grandpa") << endl; 
    // Expected: Traversal list
    cout << "DFS: " << kg.dfs("Grandpa") << endl; 
    // Expected: Traversal list

    // 7. getRelatedEntities
    cout << "\n[7] getRelatedEntities('Grandpa', depth=2):" << endl;
    vector<string> related = kg.getRelatedEntities("Grandpa", 2);
    cout << "Result: ";
    for(auto r : related) cout << r << " ";
    cout << endl;
    // Expected: Dad, Son, Daughter

    // 8. findCommonAncestors (WEIGHTED TEST)
    cout << "\n[8] findCommonAncestors() with Weights (No Cycles):" << endl;
    
    // --- Setup a new isolated graph for clear weighted testing ---
    KnowledgeGraph wKG;
    wKG.addEntity("CheapParent");
    wKG.addEntity("ExpensiveParent");
    wKG.addEntity("ChildA");
    wKG.addEntity("ChildB");

    // Scenario: 
    // Two independent parents pointing to the same children.
    // 1. ExpensiveParent connects with weight 10.0
    // 2. CheapParent connects with weight 1.0
    //
    // Graph Structure:
    // ExpensiveParent --(10)--> ChildA
    // ExpensiveParent --(10)--> ChildB
    // CheapParent     --(1)-->  ChildA
    // CheapParent     --(1)-->  ChildB
    
    wKG.addRelation("ExpensiveParent", "ChildA", 10.0f);
    wKG.addRelation("ExpensiveParent", "ChildB", 10.0f);
    
    wKG.addRelation("CheapParent", "ChildA", 1.0f);
    wKG.addRelation("CheapParent", "ChildB", 1.0f);

    cout << "  Scenario A: Parallel Parents with different weights." << endl;
    cout << "    Expensive path cost: 10 + 10 = 20" << endl;
    cout << "    Cheap path cost:      1 + 1  = 2" << endl;
    cout << "  Result: " << wKG.findCommonAncestors("ChildA", "ChildB") << endl; 
    // Expected: CheapParent

    // --- Setup Hierarchy with Shortcuts ---
    KnowledgeGraph hKG;
    hKG.addEntity("CEO");
    hKG.addEntity("Manager");
    hKG.addEntity("Intern1");
    hKG.addEntity("Intern2");

    // Graph Structure:
    // CEO --(1)--> Manager
    // Manager --(50)--> Intern1
    // Manager --(50)--> Intern2
    // CEO --(1)--> Intern1 (Direct shortcut)
    // CEO --(1)--> Intern2 (Direct shortcut)
    
    hKG.addRelation("CEO", "Manager", 1.0f);
    hKG.addRelation("Manager", "Intern1", 50.0f);
    hKG.addRelation("Manager", "Intern2", 50.0f);
    hKG.addRelation("CEO", "Intern1", 1.0f);
    hKG.addRelation("CEO", "Intern2", 1.0f);

    cout << "\n  Scenario B: Topologically distant but Weighted closer." << endl;
    cout << "    Manager is closer by hops (1 hop), but expensive (Weight 50)." << endl;
    cout << "    CEO is farther by hops (via Manager), but has direct cheap links (Weight 1)." << endl;
    cout << "    Cost via Manager: 50 + 50 = 100" << endl;
    cout << "    Cost via CEO:     1 + 1   = 2" << endl;
    cout << "  Result: " << hKG.findCommonAncestors("Intern1", "Intern2") << endl;
    cout << "  Result: " << hKG.findCommonAncestors("CEO", "Manager") << endl;

    // Expected: CEO
}

// =============================================================================
// Main
// =============================================================================
int main() {
    testEdgeClass();
    testVertexNodeClass();
    testDGraphModelClass();
    testKnowledgeGraphClass();

    cout << "\n=========================================" << endl;
    cout << "          ALL TESTS COMPLETED            " << endl;
    cout << "=========================================" << endl;
    return 0;
}