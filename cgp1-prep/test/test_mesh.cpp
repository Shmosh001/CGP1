#if HAVE_CONFIG_H
# include <config.h>
#endif

#include <test/testutil.h>
#include "test_mesh.h"
#include <stdio.h>
#include <cstdint>
#include <sstream>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>

void TestMesh::setUp()
{
    mesh = new Mesh();
}

void TestMesh::tearDown()
{
    delete mesh;
}

void TestMesh::testMeshing()
{
    CPPUNIT_ASSERT(mesh->readSTLTest("./meshes/bunny.stl"));
    CPPUNIT_ASSERT(mesh->basicValidity());
    CPPUNIT_ASSERT(!mesh->manifoldValidity()); // bunny has known holes in the bottom

    //test both basic and manifold validity - pass
    if(true)
    {
    	Mesh test;
    	CPPUNIT_ASSERT(test.readSTLTest("./meshes/Cube2.stl"));
    	CPPUNIT_ASSERT(test.basicValidity());
    	CPPUNIT_ASSERT(test.manifoldValidity());
    }

    // if(true)
    // {
    // 	Mesh test;
    // 	CPPUNIT_ASSERT(test.readSTL("./meshes/TestDanglingVerticesFail.stl"));
    // 	CPPUNIT_ASSERT(!test.danglingVertices());
    // 	CPPUNIT_ASSERT(!test.basicValidity());
    // }

    //test manifold validity - fail case
    if(true)
    {
    	Mesh test;
    	CPPUNIT_ASSERT(test.readSTLTest("./meshes/cubewithhole.stl"));
    	CPPUNIT_ASSERT(test.basicValidity());
    	CPPUNIT_ASSERT(!test.manifoldValidity());    	
    }

    //test closed manifold - pass case
    if(true)
    {
    	Mesh test;
    	CPPUNIT_ASSERT(test.readSTLTest("./meshes/TestClosed.stl"));
    	CPPUNIT_ASSERT(test.basicValidity());
    	CPPUNIT_ASSERT(!test.closedManifold());    	
    }

    //test two manifold - fail case
    if(true)
    {
    	Mesh test;
    	CPPUNIT_ASSERT(test.readSTLTest("./meshes/TwoManifoldTestFail.stl"));
    	CPPUNIT_ASSERT(test.basicValidity());
    	CPPUNIT_ASSERT(!test.twoManifoldTest());    	
    }

    //test two manifold - pass case
    if(true)
    {
    	Mesh test;
    	CPPUNIT_ASSERT(test.readSTLTest("./meshes/TwoManifoldTestPass.stl"));
    	CPPUNIT_ASSERT(test.basicValidity());
    	CPPUNIT_ASSERT(test.twoManifoldTest());    	
    }

    //test orientable - pass case
    if(true)
    {
    	Mesh test;
    	CPPUNIT_ASSERT(test.readSTLTest("./meshes/TestOrientable.stl"));
    	CPPUNIT_ASSERT(test.basicValidity());
    	CPPUNIT_ASSERT(test.windingManifold());    	
    }

    




}

//#if 0 /* Disabled since it crashes the whole test suite */
CPPUNIT_TEST_SUITE_NAMED_REGISTRATION(TestMesh, TestSet::perCommit());
//#endif
