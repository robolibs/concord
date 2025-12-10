#pragma once

#include "../../types/point.hpp"
#include "polygon.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <list>
#include <memory>
#include <vector>

namespace concord {

    enum TPPLOrientation { TPPL_ORIENTATION_CW, TPPL_ORIENTATION_CCW, TPPL_ORIENTATION_NONE };

    typedef std::list<Polygon> PolygonList;

    class TPPLPartition {
      protected:
        struct PartitionVertex {
            Point p;
            long id;
            bool isActive;
            bool isConvex;
            bool isEar;
            double angle;
            std::weak_ptr<PartitionVertex> previous;
            std::weak_ptr<PartitionVertex> next;

            PartitionVertex();
        };

        struct DPState {
            long bestvertex;
            double weight;
            bool visible;
        };

        struct Diagonal {
            long index1, index2;
        };

        typedef std::list<Diagonal> DiagonalList;

        struct DPState2 {
            long weight;
            DiagonalList pairs;
            bool visible;
        };

        // Helper functions
        Point Normalize(const Point &p);
        double Distance(const Point &p1, const Point &p2);
        int Intersects(const Point &p11, const Point &p12, const Point &p21, const Point &p22);
        bool IsConvex(const Point &p1, const Point &p2, const Point &p3);
        bool IsReflex(const Point &p1, const Point &p2, const Point &p3);
        bool IsInside(const Point &p1, const Point &p2, const Point &p3, const Point &p);
        bool InCone(const Point &p1, const Point &p2, const Point &p3, const Point &p);
        bool InCone(std::shared_ptr<PartitionVertex> v, const Point &p);
        void UpdateVertexReflexity(std::shared_ptr<PartitionVertex> v);
        void UpdateVertex(PartitionVertex *v, std::vector<std::shared_ptr<PartitionVertex>> &vertices,
                          long numvertices);
        void UpdateState(long a, long b, long w, long i, long j, std::vector<std::vector<DPState2>> &dpstates);
        void TypeA(long i, long j, long k, std::vector<std::shared_ptr<PartitionVertex>> &vertices,
                   std::vector<std::vector<DPState2>> &dpstates);
        void TypeB(long i, long j, long k, std::vector<std::shared_ptr<PartitionVertex>> &vertices,
                   std::vector<std::vector<DPState2>> &dpstates);

        // Main partitioning functions
        int Triangulate_EC(Polygon *poly, PolygonList *triangles);
        int ConvexPartition_HM(Polygon *poly, PolygonList *parts);

      public:
        int Triangulate_EC(PolygonList *inpolys, PolygonList *triangles);
        int Triangulate_OPT(Polygon *poly, PolygonList *triangles);
        int Triangulate_MONO(Polygon *poly, PolygonList *triangles);
        int ConvexPartition_HM(PolygonList *inpolys, PolygonList *parts);
        int ConvexPartition_OPT(Polygon *poly, PolygonList *parts);
        int RemoveHoles(PolygonList *inpolys, PolygonList *outpolys);
    };

} // namespace concord
