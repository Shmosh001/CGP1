//
// Mesh
//

#include "mesh.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <list>
#include <sys/stat.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/intersect.hpp>
#include <unordered_map>

using namespace std;
using namespace cgp;

GLfloat stdCol[] = {0.7f, 0.7f, 0.75f, 0.4f};
const int raysamples = 5;

bool Sphere::pointInSphere(cgp::Point pnt)
{
    cgp::Vector delvec;

    delvec.diff(c, pnt);
    if(delvec.sqrdlength() < r*r)
        return true;
    else
        return false;
}

bool Mesh::findVert(cgp::Point pnt, int &idx)
{
    bool found = false;
    int i = 0;

    idx = -1;
    // linear search of vertex list
    while(!found && i < (int) verts.size())
    {
        if(verts[i] == pnt)
        {
            found = true;
            idx = i;
        }
        i++;
    }
    return found;
}

bool Mesh::findEdge(vector<Edge> edges, Edge e, int &idx)
{
    bool found = false;
    int i = 0;

    idx = -1;
    // linear search of edge list
    while(!found && i < (int) edges.size())
    {
        if( (edges[i].v[0] == e.v[0] && edges[i].v[1] == e.v[1]) || (edges[i].v[1] == e.v[0] && edges[i].v[0] == e.v[1]) )
        {
            found = true;
            idx = i;
        }
        i++;
    }
    return found;
}


long Mesh::hashVert(cgp::Point pnt, cgp::BoundBox bbox1)
{
    long x, y, z;
    float range = 2500.0f;
    long lrangesq, lrange = 2500;

    lrangesq = lrange * lrange;

    // discretise vertex within bounds of the enclosing bounding box
    x = (long) (((pnt.x - bbox1.min.x) * range) / bbox1.diagLen()) * lrangesq;
    y = (long) (((pnt.y - bbox1.min.y) * range) / bbox1.diagLen()) * lrange;
    z = (long) (((pnt.z - bbox1.min.z) * range) / bbox1.diagLen());
    return x+y+z;
}

void Mesh::mergeVerts()
{
    vector<cgp::Point> cleanverts;
    long key;
    int i, p, hitcount = 0;
    // use hashmap to quickly look up vertices with the same coordinates
    std::unordered_map<long, int> idxlookup; // key is concatenation of vertex position, value is index into the cleanverts vector
    //cgp::BoundBox bbox;

    // construct a bounding box enclosing all vertices
    for(i = 0; i < (int) verts.size(); i++)
        bbox.includePnt(verts[i]);

    // remove duplicate vertices
    for(i = 0; i < (int) verts.size(); i++)
    {
        key = hashVert(verts[i], bbox);
        if(idxlookup.find(key) == idxlookup.end()) // key not in map
        {
            idxlookup[key] = (int) cleanverts.size(); // put index in map for quick lookup
            cleanverts.push_back(verts[i]);
        }
        else
        {
            hitcount++;
        }
    }
    cerr << "num duplicate vertices found = " << hitcount << " of " << (int) verts.size() << endl;
    cerr << "clean verts = " << (int) cleanverts.size() << endl;
    cerr << "bbox min = " << bbox.min.x << ", " << bbox.min.y << ", " << bbox.min.z << endl;
    cerr << "bbox max = " << bbox.max.x << ", " << bbox.max.y << ", " << bbox.max.z << endl;
    cerr << "bbox diag = " << bbox.diagLen() << endl;


    // re-index triangles
    for(i = 0; i < (int) tris.size(); i++)
        for(p = 0; p < 3; p++)
        {
            key = hashVert(verts[tris[i].v[p]], bbox);
            if(idxlookup.find(key) != idxlookup.end())
                tris[i].v[p] = idxlookup[key];
            else
                cerr << "Error Mesh::mergeVerts: vertex not found in map" << endl;

        }

    verts.clear();
    verts = cleanverts;
}

void Mesh::deriveVertNorms()
{
    vector<int> vinc; // number of faces incident on vertex
    int p, t;
    cgp::Vector n;

    // init structures
    for(p = 0; p < (int) verts.size(); p++)
    {
        vinc.push_back(0);
        norms.push_back(cgp::Vector(0.0f, 0.0f, 0.0f));
    }

    // accumulate face normals into vertex normals
    for(t = 0; t < (int) tris.size(); t++)
    {
        n = tris[t].n; n.normalize();
        for(p = 0; p < 3; p++)
        {
            norms[tris[t].v[p]].add(n);
            vinc[tris[t].v[p]]++;
        }
    }

    // complete average
    for(p = 0; p < (int) verts.size(); p++)
    {
        norms[p].mult(1.0f/((float) vinc[p]));
        norms[p].normalize();
    }
}

void Mesh::deriveFaceNorms()
{
    int t;
    cgp::Vector evec[2];

    for(t = 0; t < (int) tris.size(); t++)
    {
        // right-hand rule for calculating normals, i.e. counter-clockwise winding from front on vertices
        evec[0].diff(verts[tris[t].v[0]], verts[tris[t].v[1]]);
        evec[1].diff(verts[tris[t].v[0]], verts[tris[t].v[2]]);
        evec[0].normalize();
        evec[1].normalize();
        tris[t].n.cross(evec[0], evec[1]);
        tris[t].n.normalize();
    }

}

void Mesh::buildTransform(glm::mat4x4 &tfm)
{
    glm::mat4x4 idt;

    idt = glm::mat4(1.0f);
    tfm = glm::translate(idt, glm::vec3(trx.i, trx.j, trx.k));
    tfm = glm::rotate(tfm, zrot, glm::vec3(0.0f, 0.0f, 1.0f));
    tfm = glm::rotate(tfm, yrot, glm::vec3(0.0f, 1.0f, 0.0f));
    tfm = glm::rotate(tfm, xrot, glm::vec3(1.0f, 0.0f, 0.0f));
    tfm = glm::scale(tfm, glm::vec3(scale));
}

Mesh::Mesh()
{
    col = stdCol;
    scale = 1.0f;
    xrot = yrot = zrot = 0.0f;
    trx = cgp::Vector(0.0f, 0.0f, 0.0f);
}

Mesh::~Mesh()
{
    clear();
}

void Mesh::clear()
{
    verts.clear();
    tris.clear();
    geom.clear();
    col = stdCol;
    scale = 1.0f;
    xrot = yrot = zrot = 0.0f;
    trx = cgp::Vector(0.0f, 0.0f, 0.0f);
    for(int i = 0; i < (int) boundspheres.size(); i++)
        boundspheres[i].ind.clear();
    boundspheres.clear();
}

bool Mesh::genGeometry(View * view, ShapeDrawData &sdd)
{
    vector<int> faces;
    int t, p;
    glm::mat4x4 tfm;

    geom.clear();
    geom.setColour(col);

    // transform mesh data structures into a form suitable for rendering
    // by flattening the triangle list
    for(t = 0; t < (int) tris.size(); t++)
        for(p = 0; p < 3; p++)
            faces.push_back(tris[t].v[p]);

    // construct transformation matrix
    buildTransform(tfm);
    geom.genMesh(&verts, &norms, &faces, tfm);

    /*
    // generate geometry for acceleration spheres for testing
    for(p = 0; p < (int) boundspheres.size(); p++)
    {
        glm::mat4x4 idt;

        idt = glm::mat4(1.0f);
        tfm = glm::translate(idt, glm::vec3(trx.i+boundspheres[p].c.x, trx.j+boundspheres[p].c.y, trx.k+boundspheres[p].c.z));
        tfm = glm::rotate(tfm, zrot, glm::vec3(0.0f, 0.0f, 1.0f));
        tfm = glm::rotate(tfm, yrot, glm::vec3(0.0f, 1.0f, 0.0f));
        tfm = glm::rotate(tfm, xrot, glm::vec3(1.0f, 0.0f, 0.0f));
        tfm = glm::scale(tfm, glm::vec3(scale));
        geom.genSphere(boundspheres[p].r, 20, 20, tfm);
    }*/

    // bind geometry to buffers and return drawing parameters, if possible
    if(geom.bindBuffers(view))
    {
        sdd = geom.getDrawParameters();
        return true;
    }
    else
       return false;
}

void Mesh::boxFit(float sidelen)
{
    cgp::Point pnt;
    cgp::Vector shift, diag, halfdiag;
    float scale;
    int v;
    cgp::BoundBox bbox;

    // calculate current bounding box
    for(v = 0; v < (int) verts.size(); v++)
        bbox.includePnt(verts[v]);

    if((int) verts.size() > 0)
    {
        // calculate translation necessary to move center of bounding box to the origin
        diag = bbox.getDiag();
        shift.pntconvert(bbox.min);
        halfdiag = diag; halfdiag.mult(0.5f);
        shift.add(halfdiag);
        shift.mult(-1.0f);

        // scale so that largest side of bounding box fits sidelen
        scale = max(diag.i, diag.j); scale = max(scale, diag.k);
        scale = sidelen / scale;

        // shift center to origin and scale uniformly
        for(v = 0; v < (int) verts.size(); v++)
        {
            pnt = verts[v];
            shift.pntplusvec(pnt, &pnt);
            pnt.x *= scale; pnt.y *= scale; pnt.z *= scale;
            verts[v] = pnt;
        }
    }
}

bool Mesh::readSTL(string filename)
{
    ifstream infile;
    char * inbuffer;
    struct stat results;
    int insize, inpos, numt, t, i;
    cgp::Point vpos;
    Triangle tri;

    // assumes binary format STL file
    infile.open((char *) filename.c_str(), ios_base::in | ios_base::binary);
    if(infile.is_open())
    {
        clear();

        // get the size of the file
        stat((char *) filename.c_str(), &results);
        insize = results.st_size;

        // put file contents in buffer
        inbuffer = new char[insize];
        infile.read(inbuffer, insize);
        if(!infile) // failed to read from the file for some reason
        {
            cerr << "Error Mesh::readSTL: unable to populate read buffer" << endl;
            return false;
        }

        // interpret buffer as STL file
        if(insize <= 84)
        {
            cerr << "Error Mesh::readSTL: invalid STL binary file, too small" << endl;
            return false;
        }

        inpos = 80; // skip 80 character header
        if(inpos+4 >= insize){ cerr << "Error Mesh::readSTL: malformed header on stl file" << endl; return false; }
        numt = (int) (* ((long *) &inbuffer[inpos]));
        inpos += 4;

        t = 0;

        // triangle vertices have consistent outward facing clockwise winding (right hand rule)
        while(t < numt) // read in triangle data
        {
            // normal
            if(inpos+12 >= insize){ cerr << "Error Mesh::readSTL: malformed stl file" << endl; return false; }
            // IEEE floating point 4-byte binary numerical representation, IEEE754, little endian
            tri.n = cgp::Vector((* ((float *) &inbuffer[inpos])), (* ((float *) &inbuffer[inpos+4])), (* ((float *) &inbuffer[inpos+8])));
            inpos += 12;

            // vertices
            for(i = 0; i < 3; i++)
            {
                if(inpos+12 >= insize){ cerr << "Error Mesh::readSTL: malformed stl file" << endl; return false; }
                vpos = cgp::Point((* ((float *) &inbuffer[inpos])), (* ((float *) &inbuffer[inpos+4])), (* ((float *) &inbuffer[inpos+8])));
                tri.v[i] = (int) verts.size();
                verts.push_back(vpos);
                inpos += 12;
            }
            tris.push_back(tri);
            t++;
            inpos += 2; // handle attribute byte count - which can simply be discarded
        }

        // tidy up
        delete inbuffer;
        infile.close();

        cerr << "num vertices = " << (int) verts.size() << endl;
        cerr << "num triangles = " << (int) tris.size() << endl;

        // STL provides a triangle soup so merge vertices that are coincident
        mergeVerts();
        // normal vectors at vertices are needed for rendering so derive from incident faces
        deriveVertNorms();
        if(basicValidity())
            cerr << "loaded file has basic validity" << endl;
        else
            cerr << "loaded file does not pass basic validity" << endl;
    }
    else
    {
        cerr << "Error Mesh::readSTL: unable to open " << filename << endl;
        return false;
    }
    return true;
}

bool Mesh::writeSTL(string filename)
{
    ofstream outfile;
    int t, p, numt;
    unsigned short val;

    outfile.open((char *) filename.c_str(), ios_base::out | ios_base::binary);
    if(outfile.is_open())
    {
        outfile.write("File Generated by Tesselator. Binary STL", 80); // skippable header
        numt = (int) tris.size();
        outfile.write((char *) &numt, 4); // number of triangles

        for(t = 0; t < numt; t++)
        {
            // normal
            outfile.write((char *) &tris[t].n.i, 4);
            outfile.write((char *) &tris[t].n.j, 4);
            outfile.write((char *) &tris[t].n.k, 4);

            // triangle vertices
            for(p = 0; p < 3; p++)
            {
                outfile.write((char *) &verts[tris[t].v[p]].x, 4);
                outfile.write((char *) &verts[tris[t].v[p]].y, 4);
                outfile.write((char *) &verts[tris[t].v[p]].z, 4);
            }

            // attribute byte count - null
            val = 0;
            outfile.write((char *) &val, 2);
        }

        // tidy up
        outfile.close();
    }
    else
    {
        cerr << "Error Mesh::writeSTL: unable to open " << filename << endl;
        return false;
    }
    return true;
}

bool Mesh::basicValidity()
{
    
    bool isValid = true;

    //work out eulers characteristic equation
    eulersEquation();
    bool passDangling = danglingVertices();
    if(passDangling == false)
    {
        isValid = false;
    }

    manifoldValidity();

    return isValid;
}

//method for eulers characteristic equation
void Mesh::eulersEquation ()
{
    
    int numt = (int) tris.size();   //size of triangle list
    for (int x = 0; x < numt; x++)  //loop through each triangle to get vectices
    {
        int vert0 = tris[x].v[0];
        int vert1 = tris[x].v[1];
        int vert2 = tris[x].v[2];

        //vector of temporary edges
        Edge tempEdges[3];
        
        //creating 3 edges and add to vector
        Edge temp0;
        temp0.v[0] = vert0;
        temp0.v[1] = vert1;
        tempEdges[0] = temp0;

        Edge temp1;
        temp1.v[0] = vert1;
        temp1.v[1] = vert2;
        tempEdges[1] = temp1;

        Edge temp2;
        temp2.v[0] = vert2;
        temp2.v[1] = vert0;
        tempEdges[2] = temp2;

        //loop through edges
        for (int i = 0; i < 3; ++i)
        {
            if(tempEdges[i].v[0] <= tempEdges[i].v[1])
            {
                int key = tempEdges[i].v[0];
                //check if vector is empty
                if(edges[key].size() == 0)
                {
                    edges[key].push_back(verts[tempEdges[i].v[1]]);
                    uniqueEdges.push_back(tempEdges[i]);
                    
                }

                //vector has points in it
                else
                {
                    bool exists = false;
                    //loop through vector of points
                    for(int j = 0; j < edges[key].size(); ++j)
                    {
                        //check if point already exists
                        if(edges[key][j] == verts[tempEdges[i].v[1]])
                        {
                            exists = true;
                            break;
                        }
                    }

                    //if point does not exist, add point
                    if(exists == false)
                    {
                        edges[key].push_back(verts[tempEdges[i].v[1]]);
                        uniqueEdges.push_back(tempEdges[i]);
                    }
                }                
            }

            else if(tempEdges[i].v[1] < tempEdges[i].v[0])
            {
                //int key = hashVert(verts[tempEdges[i].v[1]], bbox);
                int key = tempEdges[i].v[1];
                //check if vector is empty
                if(edges[key].size() == 0)
                {
                    edges[key].push_back(verts[tempEdges[i].v[0]]);                    
                    uniqueEdges.push_back(tempEdges[i]);
                    
                }

                //vector has points in it
                else
                {
                    bool exists = false;
                    //loop through vector of points
                    for(int j = 0; j < edges[key].size(); ++j)
                    {
                        //check if point already exists
                        if(edges[key][j] == verts[tempEdges[i].v[0]])
                        {
                            exists = true;
                            break;
                        }
                    }

                    //if point does not exist, add point
                    if(exists == false)
                    {
                        edges[key].push_back(verts[tempEdges[i].v[0]]);
                        uniqueEdges.push_back(tempEdges[i]);                   
                    }
                }                
            }

        }//end loop through edges

    } //end for loop of triangles

    
    int countEdges = 0;
    for(int i = 0; i < edges.size(); i++)
    {
        countEdges = countEdges + edges[i].size();
    }
    
    cerr << "clean edges = " << countEdges << endl;

    
     //V – E + F = 2 – 2G
    int lhs = verts.size() - countEdges + tris.size();
    cerr << "Euler’s Characteristic equation lhs = " << lhs << endl;
}

//method for dangling vertices
bool Mesh::danglingVertices()
{
    bool pass = true;
    //check for dangling vertices
    int danglingVerts = 0;
    std::vector<int> vertsClone;
    //create a clone of verts vector and set all values to 0.
    for (int i = 0; i < verts.size(); ++i)
    {
        vertsClone.push_back(0);
    }

    //for every vertex in the triangle, set value at position to 1
    for (int i = 0; i < tris.size(); ++i)
    {
        vertsClone[tris[i].v[0]] = 1;
        vertsClone[tris[i].v[1]] = 1;
        vertsClone[tris[i].v[2]] = 1;
    }

    for (int i = 0; i < vertsClone.size(); ++i)
    {
        if(vertsClone[i] == 0)
        {
            danglingVerts++;
        }
    }
    

    cerr << "Dangling vertices = " << danglingVerts << endl;

    if(danglingVerts > 0)
    {
        pass = false;
    }

    return pass;
}

bool Mesh::manifoldValidity()
{
    bool isValid = true;

    bool passTwoManifold = twoManifoldTest();
    if(passTwoManifold == true)
    {
        cerr << "Two Manifold test: Pass" << endl;
    }

    else
    {
        cerr << "Two Manifold test: Fail" << endl;
    }
    
    bool passClosedManifold = closedManifold();
    if(passClosedManifold == true)
    {
        cerr << "Closed manifold test: Pass" << endl;
    }

    else
    {
        cerr << "Closed manifold test: Fail" << endl;
    }
    

    return isValid;
}

bool Mesh::twoManifoldTest()
{
    bool manifoldPass = true;
    std::vector<std::vector<Triangle>> triangleVector (verts.size());
    std::vector<std::vector<Edge>> edgeVector (verts.size());
       
    
    for (int x = 0; x < tris.size(); ++x)
    {
            int vert0 = tris[x].v[0];
            int vert1 = tris[x].v[1];
            int vert2 = tris[x].v[2];

            triangleVector[vert0].push_back(tris[x]);
            triangleVector[vert1].push_back(tris[x]);
            triangleVector[vert2].push_back(tris[x]);
    
    } //end loop tris

    for (int x = 0; x < uniqueEdges.size(); ++x)
    {
            int vert0 = uniqueEdges[x].v[0];
            int vert1 = uniqueEdges[x].v[1];
            
            edgeVector[vert0].push_back(uniqueEdges[x]);
            edgeVector[vert1].push_back(uniqueEdges[x]);

    
    } //end loop tris

    for (int i = 0; i < triangleVector.size(); ++i)
    {
        if(triangleVector[i].size() != edgeVector[i].size())
        {
            manifoldPass = false;
            break;
        }
    }


    //  // builds a hash table of hash tables. each vertex has a hash table of all the connecting vertices
    // int numt = (int) tris.size();   //size of triangle list
    // for (int x = 0; x < numt; x++)  //loop through each triangle to get vectices
    // {
    //     int vert0 = tris[x].v[0];
    //     int vert1 = tris[x].v[1];
    //     int vert2 = tris[x].v[2];

    //     //vector of temporary edges
    //     Edge tempEdges[3];
        
    //     //creating 3 edges and add to vector
    //     Edge temp0;
    //     temp0.v[0] = vert0;
    //     temp0.v[1] = vert1;
    //     tempEdges[0] = temp0;

    //     Edge temp1;
    //     temp1.v[0] = vert1;
    //     temp1.v[1] = vert2;
    //     tempEdges[1] = temp1;

    //     Edge temp2;
    //     temp2.v[0] = vert2;
    //     temp2.v[1] = vert0;
    //     tempEdges[2] = temp2;

    //     //loop through edges
    //     for (int i = 0; i < 3; ++i)
    //     {
    //         //add each vertex to a hash table
    //         linkedVerts[tempEdges[i].v[0]][tempEdges[i].v[1]] = 1;
    //         linkedVerts[tempEdges[i].v[1]][tempEdges[i].v[0]] = 1;

    //     }//end loop through edges

    // } //end for loop of triangles


    // int count = 0;
    // std::vector<std::unordered_map<int, int>> triangles;
    

    // //for each vertex
    // for(auto it0 = linkedVerts.begin(); it0 != linkedVerts.end(); ++it0)
    // {
    //     //obtain key
    //     int key0 =  it0->first;
    //     if(it0->second.size() < 3)
    //     {
    //         break;
    //     }

    //     //loop through table of all connecting vertices to linkedVerts[key]
    //     for (auto it1 = linkedVerts[key0].begin(); it1 != linkedVerts[key0].end(); ++it1)
    //     {
    //         //obtain key
    //         int key1 = it1->first;
    //         int counter = 0;
    //         for (auto it2 = linkedVerts[key1].begin(); it2 != linkedVerts[key1].end(); ++it2)
    //         {
    //             int key2 = it2->first;
    //             //check if a vertex exists
    //             if (linkedVerts[key0].find(key2) != linkedVerts[key0].end())
    //             {
    //                 count ++;
    //                 counter ++;
    //             }

    //             if(counter == 2)
    //             {
    //                 std::unordered_map<int, int> triangleVecs;
    //                 triangleVecs[key0] = 1;
    //                 triangleVecs[key1] = 1;
    //                 triangleVecs[key2] = 1;
    //                 triangles.push_back(triangleVecs);
    //                 break;
    //             }
    //         }       
    //     }
        
    //     if(count == linkedVerts[key0].size() * 2)
    //     {
    //         bool check = true;
    //         manifoldPass = true;
    //         int counterCheck = 0;
    //         for (int i = 0; i < triangles.size(); ++i)
    //         {
    //             for (int j = 0; j < tris.size(); ++j)
    //             {
    //                 if(triangles[i].find(tris[j].v[0]) != triangles[i].end() && triangles[i].find(tris[j].v[1]) != triangles[i].end() && triangles[i].find(tris[j].v[2]) != triangles[i].end())
    //                 {
    //                     check = true;
    //                     counterCheck ++;

    //                     break;
    //                 }
                    
    //             }
                
    //         }
    //         if(counterCheck != triangles.size())
    //         {
    //             manifoldPass = false;
    //         }
            
    //     }

    //     else
    //     {
    //         manifoldPass = false;
    //         break;
    //     }
    //     count = 0;
    // } //end vertex loop

    return manifoldPass;
}

//check if every edge has 2 adjacent triangles
bool Mesh::closedManifold()
{
    bool pass = true;

    int numt = (int) tris.size();   //size of triangle list
    for (int x = 0; x < numt; x++)  //loop through each triangle to get vertices
    {
        int vert0 = tris[x].v[0];
        int vert1 = tris[x].v[1];
        int vert2 = tris[x].v[2];

        //vector of temporary edges
        Edge tempEdges[3];
        
        //creating 3 edges and add to vector
        Edge temp0;
        temp0.v[0] = vert0;
        temp0.v[1] = vert1;
        tempEdges[0] = temp0;

        Edge temp1;
        temp1.v[0] = vert1;
        temp1.v[1] = vert2;
        tempEdges[1] = temp1;

        Edge temp2;
        temp2.v[0] = vert2;
        temp2.v[1] = vert0;
        tempEdges[2] = temp2;

        //loop through edges
        for (int i = 0; i < 3; ++i)
        {
            if(tempEdges[i].v[0] <= tempEdges[i].v[1])
            {
                edgeList[tempEdges[i].v[0]].push_back(verts[tempEdges[i].v[1]]);
                               
            }

            else if(tempEdges[i].v[1] < tempEdges[i].v[0])
            {
                edgeList[tempEdges[i].v[1]].push_back(verts[tempEdges[i].v[0]]);                
            }

        }//end loop through edges

    } //end for loop of triangles


    for(auto it = edgeList.begin(); it != edgeList.end(); ++it)
    {
        cerr << "Closed" << it->second.size() << endl;
        if(it->second.size() != 2)
        {
            pass = false;
            break;
        }
    }

    return pass;
}

