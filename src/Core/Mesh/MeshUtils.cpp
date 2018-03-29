#include <Core/Mesh/MeshUtils.hpp>

#include <utility>
#include <set>
#include <map>

#include <Core/Math/Math.hpp>
#include <Core/Math/RayCast.hpp>
#include <Core/String/StringUtils.hpp>

namespace Ra
{
    namespace Core
    {
        namespace MeshUtils
        {
            void getAutoNormals( const TriangleMesh& mesh, VectorArray<Vector3>& normalsOut )
            {
                const uint numVertices = mesh.m_vertices.size();
                const uint numTriangles = mesh.m_triangles.size();

                normalsOut.clear();
                normalsOut.resize( numVertices, Vector3::Zero() );

                for ( uint t = 0; t < numTriangles; t++ )
                {
                    const Triangle& tri = mesh.m_triangles[t];
                    Vector3 n = getTriangleNormal( mesh, t );

                    for ( uint i = 0; i < 3; ++i )
                    {
                        normalsOut[tri[i]] += n;
                    }
                }

                normalsOut.getMap().colwise().normalize();
            }


            bool findDuplicates( const TriangleMesh& mesh, std::vector<VertexIdx>& duplicatesMap )
            {
                bool hasDuplicates = false;
                duplicatesMap.clear();
                const uint numVerts = mesh.m_vertices.size();
                duplicatesMap.resize( numVerts,VertexIdx(-1) );

                VectorArray<Vector3>::const_iterator vertPos;
                VectorArray<Vector3>::const_iterator duplicatePos;
                std::vector<std::pair<Vector3, VertexIdx>> vertices;

                for ( uint i = 0; i < numVerts; ++i )
                {
                    vertices.push_back(std::make_pair(mesh.m_vertices[i], VertexIdx(i)));
                }

                std::sort(vertices.begin(), vertices.end(),
                          [](std::pair<Vector3, int>a , std::pair<Vector3, int>b){
                              if(a.first.x() == b.first.x())
                              {
                                  if(a.first.y() == b.first.y())
                                      if(a.first.z() == b.first.z())
                                          return a.second < b.second;
                                      else
                                          return a.first.z() < b.first.z();
                                  else
                                      return a.first.y() < b.first.y();
                              }
                              return a.first.x() < b.first.x();
                          }
                    );
                // Here vertices contains vertex pos and idx, with equal
                // vertices contiguous, sorted by idx, so checking if current
                // vertex equals the previous one state if its a duplicated
                // vertex position.
                duplicatesMap[vertices[0].second] = vertices[0].second;
                for ( uint i = 1; i < numVerts; ++i )
                {
                    if(vertices[i].first == vertices[i-1].first)
                    {
                        duplicatesMap[vertices[i].second] = duplicatesMap[vertices[i-1].second];
                        hasDuplicates = true;
                    }
                    else{
                        duplicatesMap[vertices[i].second] = vertices[i].second;
                    }
                }

                return hasDuplicates;
            }

            void removeDuplicates(TriangleMesh& mesh, std::vector<VertexIdx>& vertexMap)
            {
                std::vector<VertexIdx> duplicatesMap;
                findDuplicates(mesh, duplicatesMap);

                std::vector<VertexIdx> newIndices(mesh.m_vertices.size(), VertexIdx(-1));
                Vector3Array uniqueVertices;
                Vector3Array uniqueNormals;
                for (uint i = 0; i < mesh.m_vertices.size(); i++)
                {
                    if (duplicatesMap[i] == i)
                    {
                        newIndices[i] = uniqueVertices.size();
                        uniqueVertices.push_back(mesh.m_vertices[i]);
                        uniqueNormals.push_back(mesh.m_normals[i]);
                    }
                }

                for (uint i = 0; i < mesh.m_triangles.size(); i++)
                {
                    auto &t = mesh.m_triangles[i];
                    for (uint j = 0; j < 3; j++)
                    {
                        t(j) = newIndices[ duplicatesMap[ t(j) ] ];
                    }
                }

                for (uint i = 0; i < mesh.m_faces.size(); i++)
                {
                    auto &f = mesh.m_faces[i];
                    for (uint j = 0; j < f.size(); j++)
                    {
                        f(j) = newIndices[ duplicatesMap[ f(j) ] ];
                    }
                }

                vertexMap.resize(mesh.m_vertices.size());
                for (uint i = 0; i < mesh.m_vertices.size(); i++)
                    vertexMap[i] = newIndices[duplicatesMap[i]];

                mesh.m_vertices = uniqueVertices;
                mesh.m_normals  = uniqueNormals;
            }

            RayCastResult castRay(const TriangleMesh &mesh, const Ray &ray)
            {
                RayCastResult result;
                result.m_hitTriangle = -1;
                Scalar minT = std::numeric_limits<Scalar>::max();

                std::vector<Scalar> tValues;
                std::array<Vector3,3> v;
                for ( uint i = 0; i < mesh.m_triangles.size(); ++i)
                {
                    tValues.clear();
                    getTriangleVertices(mesh, i, v);
                    if ( RayCast::vsTriangle(ray, v[0], v[1], v[2], tValues) && tValues[0] < minT )
                    {
                        minT = tValues[0];
                        result.m_hitTriangle = int(i);
                    }
                }

                if (result.m_hitTriangle >= 0)
                {
                    Scalar minDist = std::numeric_limits<Scalar>::max();
                    std::array<Vector3,3> v;
                    getTriangleVertices(mesh, result.m_hitTriangle, v);
                    for (uint i = 0; i < 3; ++i)
                    {
                        Scalar dSq = (v[i] - ray.pointAt(minT)).squaredNorm();
                        if (dSq < minDist)
                        {
                            result.m_nearestVertex = mesh.m_triangles[result.m_hitTriangle][i];
                        }
                    }
                    result.m_t = minT;

                }

                return result;
            }

            /// Return the mean edge length of the given triangle mesh
            Scalar getMeanEdgeLength( const TriangleMesh& mesh ) {
                typedef std::pair< uint, uint > Key;
                std::set< Key > list;
                const uint size = mesh.m_triangles.size();
                uint   edgeSize   = 0;
                Scalar edgeLength = 0.0;
                #pragma omp parallel for
                for( int t = 0; t < int(size); ++t ) {
                    for( uint v = 0; v < 3; ++v ) {
                        const uint i = mesh.m_triangles[t][v];
                        const uint j = mesh.m_triangles[t][( v + 1 ) % 3];
                        Key k( ( ( i < j ) ? i : j ), ( ( i < j ) ? j : i ) );
                        Scalar length = ( mesh.m_vertices[i] - mesh.m_vertices[j] ).norm();
                        #pragma omp critical
                        {
                            auto it = list.find( k );
                            if( it == list.end() ) {
                                list.insert( k );
                                ++edgeSize;
                                edgeLength += length;
                            }
                        }
                    }
                }
                if( edgeSize != 0 ) {
                    return ( edgeLength / Scalar( edgeSize ) );
                }
                return 0.0;
            }


            void checkConsistency( const TriangleMesh& mesh )
            {
#ifdef CORE_DEBUG
                std::vector<bool> visited( mesh.m_vertices.size(), false );
                for ( uint t = 0 ; t < mesh.m_triangles.size(); ++t )
                {
                    std::string errStr;
                    CORE_WARN_IF( !(getTriangleArea( mesh, t ) > 0.f), "Triangle" << t << " is degenerate" );
                    const Triangle& tri = mesh.m_triangles[t];
                    for ( uint i = 0; i < 3; ++i )
                    {
                        CORE_ASSERT( uint( tri[i] ) < mesh.m_vertices.size(),
                            "Vertex "<< tri[i] <<" is in triangle "<< t <<" (#"<< i <<") is out of bounds");
                        visited[tri[i]] = true;
                    }
                }

                for ( uint v = 0; v < visited.size(); ++v )
                {
                    std::string errStr;
                    CORE_ASSERT( visited[v], "Vertex "<< v <<" does not belong to any triangle");
                }

                // Normals are optional but if they are present then every vertex should have one.
                CORE_ASSERT( mesh.m_normals.size() ==  0 || mesh.m_normals.size() == mesh.m_vertices.size(),
                             "Inconsistent number of normals" );
#endif
            }

            void bakeTransform(TriangleMesh &mesh, const Transform &T)
            {
                for (auto& v : mesh.m_vertices)
                {
                    v = T*v;
                }
                for (auto& n : mesh.m_normals)
                {
                    n = T.rotation() * n;
                }
            }

            // See "Efficient feature extraction for 2D/3D objects in mesh representation"
            // Cha Zhang and Tsuhan Chen, 2001
            Scalar getVolume(const TriangleMesh &mesh, const Vector3 &origin)
            {
                Scalar volume {0};
                for (const auto& t : mesh.m_triangles)
                {
                    Vector3 a = mesh.m_vertices[t[0]] - origin;
                    Vector3 b = mesh.m_vertices[t[1]] - origin;
                    Vector3 c = mesh.m_vertices[t[2]] - origin;

                    volume += (a.dot(b.cross(c)));
                }
                return std::abs(volume) / Scalar (6);
            }



        } // namespace MeshUtils
    } // namespace Core
} // namespace Ra
