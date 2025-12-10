#include "KnowledgeGraph.h"

// =============================================================================
// Simple Testing Framework
// =============================================================================
int passedTests = 0;
int totalTests = 0;

template<typename T>
void assertEqual(T actual, T expected, string testName) {
    totalTests++;
    if (actual == expected) {
        cout << "[PASS] " << testName << endl;
        passedTests++;
    } else {
        cout << "[FAIL] " << testName << endl;
        cout << "       Expected: " << expected << endl;
        cout << "       Actual:   " << actual << endl;
    }
}

void assertTrue(bool condition, string testName) {
    assertEqual(condition, true, testName);
}

// Helper to format output for testing
string int2str(int& i) { return to_string(i); }
bool intEQ(int& a, int& b) { return a == b; }

// =============================================================================
// Test Scenarios
// =============================================================================

void testLowLevelGraph() {
    cout << "\n=== Testing Low-Level DGraphModel (Integer Graph) ===" << endl;
    
    DGraphModel<int> intGraph(intEQ, int2str);

    // 1. Test add and contains
    intGraph.add(1);
    intGraph.add(2);
    intGraph.add(3);
    assertTrue(intGraph.contains(1), "Graph contains vertex 1");
    assertTrue(!intGraph.contains(99), "Graph does not contain 99");

    // 2. Test connect and weights
    intGraph.connect(1, 2, 5.5f);
    intGraph.connect(2, 3, 2.0f);
    assertTrue(intGraph.connected(1, 2), "Vertex 1 connected to 2");
    assertTrue(!intGraph.connected(1, 3), "Vertex 1 not directly connected to 3");
    assertEqual(intGraph.weight(1, 2), 5.5f, "Edge weight 1->2 is 5.5");

    // 3. Test Degrees
    // 1 -> 2 -> 3
    assertEqual(intGraph.outDegree(1), 1, "Vertex 1 out-degree");
    assertEqual(intGraph.inDegree(2), 1, "Vertex 2 in-degree");
    assertEqual(intGraph.outDegree(2), 1, "Vertex 2 out-degree");
    assertEqual(intGraph.inDegree(3), 1, "Vertex 3 in-degree");

    // 4. Test Update Weight
    intGraph.connect(1, 2, 10.0f); // Update existing
    assertEqual(intGraph.weight(1, 2), 10.0f, "Updated edge weight 1->2");

    // 5. Test Disconnect
    intGraph.disconnect(1, 2);
    assertTrue(!intGraph.connected(1, 2), "Vertex 1 disconnected from 2");
    assertEqual(intGraph.outDegree(1), 0, "Vertex 1 out-degree after disconnect");

    // 6. Test Size and Empty
    assertEqual(intGraph.size(), 3, "Graph size is 3 nodes");
    assertTrue(!intGraph.empty(), "Graph is not empty");
    
    intGraph.clear();
    assertTrue(intGraph.empty(), "Graph is empty after clear");
}

void testKnowledgeGraphBasics() {
    cout << "\n=== Testing KnowledgeGraph Wrapper ===" << endl;
    
    KnowledgeGraph kg;

    // 1. Add Entities
    kg.addEntity("Alice");
    kg.addEntity("Bob");
    kg.addEntity("Charlie");
    kg.addEntity("David");

    try {
        kg.addEntity("Alice");
        cout << "[FAIL] Should have thrown EntityExistsException" << endl;
    } catch (...) {
        cout << "[PASS] Threw Exception on duplicate entity" << endl;
    }

    // 2. Add Relations
    kg.addRelation("Alice", "Bob", 1.0);
    kg.addRelation("Bob", "Charlie", 2.0);
    kg.addRelation("Alice", "David", 1.0);

    // 3. Test Neighbors
    vector<string> neighbors = kg.getNeighbors("Alice");
    // Neighbors could be in any order, so we check size and content manually or just size for simplicity here
    assertEqual((int)neighbors.size(), 2, "Alice has 2 neighbors"); 

    // 4. Test Reachability
    assertTrue(kg.isReachable("Alice", "Charlie"), "Alice -> Charlie is reachable");
    assertTrue(!kg.isReachable("Charlie", "Alice"), "Charlie -> Alice is NOT reachable");
}

void testTraversals() {
    cout << "\n=== Testing BFS and DFS ===" << endl;
    DGraphModel<int> g(intEQ, int2str);
    g.add(1); g.add(2); g.add(3); g.add(4);
    
    // Structure: 1 -> 2, 1 -> 3, 2 -> 4
    g.connect(1, 2, 1);
    g.connect(1, 3, 1);
    g.connect(2, 4, 1);

    string bfsResult = g.BFS(1);
    // BFS from 1 should visit 1, then 2/3, then 4. Exact order depends on internal storage.
    // We check if it starts correctly.
    cout << "BFS Result: " << bfsResult << endl;
    assertTrue(bfsResult.length() > 5, "BFS returned non-empty string");

    string dfsResult = g.DFS(1);
    cout << "DFS Result: " << dfsResult << endl;
    assertTrue(dfsResult.length() > 5, "DFS returned non-empty string");
}

void testRelatedEntities() {
    cout << "\n=== Testing getRelatedEntities (Depth Limit) ===" << endl;
    KnowledgeGraph kg;
    kg.addEntity("A"); kg.addEntity("B"); kg.addEntity("C"); kg.addEntity("D");
    
    // A -> B -> C -> D
    kg.addRelation("A", "B", 1);
    kg.addRelation("B", "C", 1);
    kg.addRelation("C", "D", 1);

    // Depth 1 from A -> Should get B
    vector<string> depth1 = kg.getRelatedEntities("A", 1);
    assertEqual((int)depth1.size(), 1, "Depth 1 from A has 1 entity");
    if(!depth1.empty()) assertEqual(depth1[0], string("B"), "Depth 1 entity is B");

    // Depth 2 from A -> Should get B, C
    vector<string> depth2 = kg.getRelatedEntities("A", 2);
    assertEqual((int)depth2.size(), 2, "Depth 2 from A has 2 entities");
}

void testCommonAncestors() {
    cout << "\n=== Testing findCommonAncestors Logic ===" << endl;
    KnowledgeGraph kg;
    
    // Setup a hierarchy:
    //       Grandparent
    //       /        \ (weight 10)
    //    Parent1    Parent2
    //       \      /
    //        \    /
    //         Child
    //
    // Note: This tests the direction. Usually Ancestor -> Child.
    // To find common ancestor of two nodes, we need a structure like:
    //       Ancestor
    //       /      \
    //    Child1   Child2
    
    kg.addEntity("Root");
    kg.addEntity("Inter1");
    kg.addEntity("Inter2");
    kg.addEntity("LeafA");
    kg.addEntity("LeafB");

    // Case 1: Simple Common Parent
    // Root -> LeafA
    // Root -> LeafB
    kg.addRelation("Root", "LeafA", 5.0);
    kg.addRelation("Root", "LeafB", 5.0);
    
    string ancestor = kg.findCommonAncestors("LeafA", "LeafB");
    assertEqual(ancestor, string("Root"), "Simple common ancestor is Root");

    // Case 2: Best Ancestor by Weight
    // Root(100) -> Inter1(1) -> LeafA
    //             Inter1(1) -> LeafB
    // Total Path via Root: 100+1 + 100+1 = 202
    // Total Path via Inter1: 1 + 1 = 2
    
    kg.addEntity("BadRoot");
    kg.addRelation("BadRoot", "LeafA", 100.0);
    kg.addRelation("BadRoot", "LeafB", 100.0);
    
    // Now LeafA and LeafB have common ancestors: Root (dist 5+5=10) and BadRoot (dist 200).
    ancestor = kg.findCommonAncestors("LeafA", "LeafB");
    assertEqual(ancestor, string("Root"), "Should choose closer ancestor (Root over BadRoot)");

    // Case 3: No Common Ancestor
    kg.addEntity("Alien1");
    kg.addEntity("Alien2");
    string noAnc = kg.findCommonAncestors("Alien1", "Alien2");
    assertEqual(noAnc, string("No common ancestor"), "Disconnected nodes have no ancestor");
}

int main() {
    try {
        testLowLevelGraph();
        testKnowledgeGraphBasics();
        testTraversals();
        testRelatedEntities();
        testCommonAncestors();

        cout << "\n=========================================" << endl;
        cout << "Test Summary: " << passedTests << "/" << totalTests << " passed." << endl;
        cout << "=========================================" << endl;
        
    } catch (const exception& e) {
        cout << "CRITICAL FAILURE: Uncaught exception in main: " << e.what() << endl;
    }
    return 0;
}