/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _POINTS_H_
#define _POINTS_H_

class FileInfo;

struct Point
{
	osg::Vec3 coordinates;
};

struct Color
{
    float r;
    float g;
    float b;
};

struct PointSet
{
    int size = 0;
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;
    Point *points = nullptr;
    Color *colors = nullptr;
    uint32_t *IDs = nullptr;

    PointSet()
        : IDs(nullptr), colors(nullptr), points(nullptr), size(0)
        { }
    ~PointSet()
        {
            if (points != nullptr)
            {
                delete[] points;
            }
            if (colors != nullptr)
            {
                delete[] colors;
            }
            if (IDs != nullptr)
            {
                delete[] IDs;
            }
        }
};

struct pointSelection
{
    const FileInfo *file;
    int pointSetIndex;
    int pointIndex;
    osg::MatrixTransform *transformationMatrix;
    int selectionIndex;
    bool isBoundaryPoint;    
};

struct ScannerPosition
{
    int type = 0; // 0: original imported, 1: moved, 2: copy
    uint32_t ID;
    osg::Vec3 point;
};

#endif
